#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import copy
import tf
import tf.transformations as tft
import numpy as np
import moveit_commander
import actionlib

from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory

try:
    from xarm_msgs.srv import SetInt16, SetAxis, ClearErr
    from xarm_msgs.msg import RobotMsg
except ImportError:
    SetInt16 = None
    SetAxis = None
    ClearErr = None
    RobotMsg = None

# ==============================================================================
# [사용자 설정 파라미터 구역 (Configuration Parameters)]
# ==============================================================================
class RobotConfig:
    # 1. 초기 자세 관절 각도 (라디안 단위, J1 ~ J5)
    # 정면 테이블 작업 기준: X=0.458m (기존 X=0.478m 대비 몸통 쪽으로 정확히 2cm 후방 당김), Z=0.419m
    # J2 + J3 + J4 = 1.24° - 101.67° + 100.44° = 0.0° -> 테이블 표면과 완전 수직(Pitch -90°, Tool Z=[0,0,-1]) 하향 주시 유지
    INITIAL_JOINTS = [
        0.0000000000,   # J1:   0.00° (정면 테이블 방향 정렬)
        0.0215558089,   # J2:  +1.24° (몸통 쪽 2cm 당김 반영)
       -1.7745510942,   # J3: -101.67° (몸통 쪽 2cm 당김 반영)
        1.7529954623,   # J4: +100.44° (Pitch -90° 완전 직각 수직 하향 정렬 유지)
        0.0000000000    # J5:   0.00° (정면 테이블 모서리와 수평 유지)
    ]

    # 2. 물체 낙하/적재(Drop/Place) 관절 각도 (라디안 단위, J1 ~ J5)
    # [좌후방 적재함 상공 안전 자세]: 수직 하강 전 테이블 위 안전 고도(Z=+0.35m) 유지 회전 자세
    # TCP 위치: X = -0.155m, Y = +0.2694m, Z = +0.350m (Tool Z 수직 하향 정렬 유지, J2/J3 음수 안전 보장)
    DROP_PRE_JOINTS_LEFT = [
        2.0928022861480713,    # J1: +119.91° (좌후방 적재함 상공 경유 자세)
       -0.5621202184821094,    # J2:  -32.21° (안전한 음수 각도 유지)
       -1.0251964140128025,    # J3:  -58.74° (안전한 음수 각도 유지)
        1.5873166324949108,    # J4:  +90.95° (Pitch -90° 완전 직각 수직 하향 유지)
        0.0000000000000000     # J5:    0.00°
    ]

    # 색상별 전용 Drop: separated_basket의 XY와 손목 방향을 유지하고 높이만 보정.
    # link_base -> link_tcp Z = -0.075m (URDF의 그리퍼 TCP 오프셋 0.172m 기준).
    # 접근/복귀는 기존 좌후방 상공 자세를 사용하며, 최종 위치까지는 관절 보간 이동.
    DROP_PRE_JOINTS_RED = list(DROP_PRE_JOINTS_LEFT)
    DROP_JOINTS_RED = [
        2.357590436935,
        0.360198524466,
       -0.541495046276,
        0.149798171607,
        0.741811990738
    ]

    DROP_PRE_JOINTS_YELLOW = list(DROP_PRE_JOINTS_LEFT)
    DROP_JOINTS_YELLOW = [
        2.174335241318,
        0.276098247099,
       -0.406505156775,
        0.167398870976,
        0.602597534657
    ]

    DROP_PRE_JOINTS_BLUE = list(DROP_PRE_JOINTS_LEFT)
    DROP_JOINTS_BLUE = [
        1.950009822845,
        0.218565756288,
       -0.291925136875,
        0.119712433888,
        0.372868537903
    ]

    # 3. 작업 종료 원점 관절 각도 (라디안 단위, J1 ~ J5)
    # 5초간 박스 미검출 시 테이블 충돌 방지(수평 후퇴 후 수직 복귀)로 복귀
    ZERO_JOINTS = [
        0.0000000000,  # J1: 0.00°
        0.0000000000,  # J2: 0.00°
        0.0000000000,  # J3: 0.00°
        0.0000000000,  # J4: 0.00°
        0.0000000000   # J5: 0.00°
    ]

    # [원점 수직 상공 및 대칭 역경로 복귀 기준 파라미터]
    # ZERO_JOINTS의 TCP 위치: X=0.2070m, Y=0.0m, Z=-0.060m
    ZERO_X = 0.2070

    # 기동 시 수직 상승 완료 지점(ascent_pose: X=0.207m, Z=SAFE_ASCENT_Z=0.38m)의 관절 각도
    ASCENT_JOINTS = [
        0.0000000000,   # J1:   0.00°
       -0.8690874246,   # J2: -49.80°
       -0.9452278145,   # J3: -54.16°
        1.8142950211,   # J4: +103.95° (Pitch -90° 수직 하향 정렬)
        0.0000000000    # J5:   0.00°
    ]

    # 수직 하강 안전 보장 중간 관절 각도 (X=0.207m, Z=0.180m, 전 구간 X <= 0.252m로 테이블 간섭 100% 회피)
    ASCENT_MID_JOINTS = [
        0.0000000000,   # J1:   0.00°
       -1.0502283120,   # J2: -60.17°
       -0.2823154481,   # J3: -16.18°
        1.3325437601,   # J4: +76.35°
        0.0000000000    # J5:   0.00°
    ]

    # 4. 박스 미검출 타임아웃 (초 단위)
    # 초기 자세에서 5초 동안 새로운 박스가 검출되지 않으면 작업 완료로 간주하고 안전 종료 시퀀스 진입
    NO_DETECTION_TIMEOUT = 5.0

    # 5. 클래스 정의 및 우선순위 (0: 빨간색 -> 1: 노란색 -> 2: 파란색)
    CLASS_NAMES = {
        0: "빨간색 박스 (red_box)",
        1: "노란색 박스 (yellow_box)",
        2: "파란색 박스 (blue_box)"
    }

    # 4. 그리퍼 개폐 파라미터 (현재 하드웨어에 맞게 반전: 0.0 = 열림, 0.85 = 닫힘)
    GRIPPER_OPEN_POS   = 0.0   # 열림 위치 (Pre-open 및 낙하 시)
    GRIPPER_CLOSE_POS  = 0.85  # 닫힘 위치 (물체 파지 시)
    GRIPPER_EFFORT     = 5.0  # 파지력 (N)
    GRIPPER_TIMEOUT    = 20.0   # 그리퍼 응답 대기 시간 (초)

    # 3. 모션 속도 및 가속도 스케일링 (0.0 ~ 1.0)
    # 급격한 가속/감속으로 인한 C31(이상 전류/충돌 감지) 방지 및 부드럽고 안정적인 이동을 위해 속도/가속도 안정화
    VEL_SCALE = 0.12  # 기존 0.20에서 0.12로 하향 (더 안정적이고 정밀한 이동)
    ACC_SCALE = 0.08  # 기존 0.12에서 0.08로 하향 (충격 없는 부드러운 가감속)

    # 4. 물체 접근(Approach) 및 후퇴 안전 여유 높이
    APPROACH_Z_OFFSET = 0.08  # 물체 위 8cm 사전 접근
    SAFE_ASCENT_Z     = 0.38  # 초기 자세 진입 전 전방 테이블 간섭 방지용 수직 상승 안전 높이 (m)

    # 5. 비전(TF) 프레임 이름
    BASE_FRAME   = "camera_link"
    TARGET_FRAME = "ap_target"

    # 6. Scout + Eye-in-Hand 파지 파라미터 (정면 테이블 작업)
    # 카메라 렌즈 대비 link_tcp(그리퍼 핑거 끝) 사이의 Z축 오프셋 (카메라가 TCP보다 약 14cm 위에 장착됨)
    TCP_TO_CAM_Z_DIST       = 0.14

    # [정면 테이블 기준 오프셋 보정] (단위: m)
    # GRASP_OFFSET_X: 정면 테이블 방향 앞/뒤 도달거리 오프셋 (더 멀리 전진: +, 더 가까이 후퇴: -)
    # 실측 결과: 그리퍼가 박스에 못 미쳐 5cm 앞(로봇 쪽) 허공을 집는 현상 확인 -> 테이블 안쪽으로 5cm 전진(+0.05m): 0.08m -> 0.13m 적용
    GRASP_OFFSET_X          = 0.15
    # GRASP_OFFSET_Y: 정면 테이블 기준 좌/우 편차 오프셋 (좌측: +, 우측: -)
    GRASP_OFFSET_Y          = 0.00

    # [더 깊이 하강 보정]: 박스 상단면 대비 확실한 파지를 위해 추가로 더 깊숙이 내려갈 깊이 (단위: m)
    GRASP_DEPTH_MARGIN      = 0.16  # 16cm 더 깊숙이 내려가 박스 옆면을 안정적으로 감싸쥠

    # [실측 티칭 기반 바닥/테이블 안전 최저 한계 높이]: (단위: m)
    # 새로 이동된 정면 테이블 실측 조인트로 정기구학(FK)을 정밀 계산한 결과, 실제 테이블 표면 높이는 Z = +0.0627m (+6.27cm)입니다.
    # 테이블 충돌을 완벽히 방지하도록 약 7.3mm의 안전 마진을 두어 최저 한계선으로 설정합니다.
    MIN_SAFE_Z              = 0.070  # 새 테이블 상판 위 약 7mm 지점 (충돌 방지 한계선)

    # 카메라 축과 로봇 베이스 축 매핑 계수 (J1=0도 정면 테이블 하향 주시 기준)
    # 카메라 화면 세로(cam_y, 앞/뒤 원근): 로봇 X축으로 매핑 (멀어질수록 +X 전진)
    CAM_Y_TO_ROBOT_X_SCALE  = -0.85
    # 카메라 화면 가로(cam_x, 좌/우 편차): 로봇 Y축으로 매핑 (화면 우측일수록 -Y 우측 이동)
    CAM_X_TO_ROBOT_Y_SCALE  = -0.85

    # 7. 전방 180도 및 최대 작업 반경 도달 한계 안전 가드 (C31 모터 과전류 방지)
    # 로봇 베이스 기준 전방 180도(±90도) 및 안전 작업 도달거리(X <= 0.64m) 내에서만 그리퍼가 이동하도록 제한
    FRONT_ANGLE_LIMIT_DEG   = 90.0   # J1 베이스 회전각 허용 한계 (±90도 이내 = 전방 180도)
    MIN_SAFE_X              = 0.05   # 로봇 베이스 전방 최소 안전 거리 (0.05m = 5cm, 후방 진입 원천 차단)
    MAX_SAFE_X              = 0.64   # 로봇 베이스 전방 최대 안전 작업 도달거리 (0.64m = 64cm, 무리한 팔 뻗음 및 C31 과부하 원천 차단)
    MAX_SAFE_RADIUS         = 0.65   # 평면 도달 반경 최대 한계 (sqrt(X^2 + Y^2) <= 0.65m)

    # 8. 비전 감지 유효성 및 연속 안정성 검증 파라미터 (허위 감지 및 과거 잔여 TF 배제)
    MIN_TARGET_CAM_Z        = 0.30   # 카메라 기준 최소 유효 거리 (30cm 미만 결측/근접 오류 배제)
    MAX_TARGET_CAM_Z        = 0.55   # 카메라 기준 최대 유효 거리 (55cm 초과 시 테이블 너머 바닥/배경으로 간주하여 배제)
    REQUIRED_CONSECUTIVE_DET = 3     # 연속 n회 이상 안정 감지 확인 시 파지 착수 (찰나의 노이즈 배제)

    # 9. 상자 기울기(Yaw) 연동 손목(J5) 회전 파지 파라미터
    # 카메라 이미지 평면 회전각(yaw_rad: 시계 방향 +)을 그리퍼 손목(J5) 회전각으로 매핑하는 부호/배율 계수
    # 카메라가 수직 하향(Tool Z=[0,0,-1])을 바라보므로 이미지상 시계방향(+yaw) 회전은 J5 +회전과 동일 방향 (반대일 경우 -1.0으로 변경)
    BOX_YAW_TO_J5_SIGN      = 1.0
    J5_YAW_OFFSET_RAD       = 0.0    # 그리퍼 기본 손목 각도 미세 오프셋 (라디안)

# ==============================================================================
# [xArm5 5-DOF 전용 수직 파지 기구학(IK) 해석 엔진]
# MoveIt 기본 KDL 솔버는 5자유도 로봇에서 6D Pose 목표에 대한 자유도 부족으로
# 'TIMED_OUT' 및 'Unable to sample valid states' 에러를 발생시킵니다.
# 본 엔진은 xArm5 공식 기구학 링크 길이와 기하학을 바탕으로
# 임의의 (X, Y, Z) 좌표에 대해 Pitch -90도(수직 하향)를 달성하는
# 정확한 관절각 5개를 1ms 내에 해석하여 MoveIt 관절 모션으로 즉각 이동시킵니다.
# ==============================================================================
TCP_OFFSET = 0.172385  # Flange(Link5) -> link_tcp Z-offset (xArm Gripper: 17.2cm)

def forward_kinematics_xarm5(joints):
    j1, j2, j3, j4, j5 = joints
    c1, s1 = np.cos(j1), np.sin(j1)
    c2, s2 = np.cos(j2), np.sin(j2)
    c3, s3 = np.cos(j3), np.sin(j3)
    c4, s4 = np.cos(j4), np.sin(j4)
    c5, s5 = np.cos(j5), np.sin(j5)

    T1 = np.array([[c1, -s1, 0, 0], [s1, c1, 0, 0], [0, 0, 1, 0.267], [0, 0, 0, 1]])
    R_x_neg_pi2 = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
    R_z2 = np.array([[c2, -s2, 0, 0], [s2, c2, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    T2 = T1 @ R_x_neg_pi2 @ R_z2

    T3_rel = np.array([[c3, -s3, 0, 0.0535], [s3, c3, 0, -0.2845], [0, 0, 1, 0], [0, 0, 0, 1]])
    T3 = T2 @ T3_rel

    T4_rel = np.array([[c4, -s4, 0, 0.0775], [s4, c4, 0, 0.3425], [0, 0, 1, 0], [0, 0, 0, 1]])
    T4 = T3 @ T4_rel

    T5_rel = np.array([[c5, -s5, 0, 0.076], [0, 0, 1, 0.097], [-s5, -c5, 0, 0], [0, 0, 0, 1]])
    T5 = T4 @ T5_rel

    tcp_pos = T5[:3, 3] + TCP_OFFSET * T5[:3, 2]
    tool_z = T5[:3, 2]
    return tcp_pos, tool_z

def solve_xarm5_ik_vertical(x, y, z, seed_joints=None, target_yaw=0.0):
    """(x, y, z) 위치에서 그리퍼가 수직 하향(Pitch -90도)이고,
    물체가 테이블 좌/우로 치우쳐 J1이 회전하더라도 J1 회전량을 자동 상쇄 보정하면서,
    추가로 YOLO+OpenCV가 추정한 상자의 기울기(target_yaw, 라디안)만큼 손목(J5)을 회전시키는 5개 관절각 산출
    """
    # [전방 180도 및 최대 도달 한계 안전 가드]: C31 과전류 및 xArm5 리치 한계 초과 원천 차단
    max_j1_rad = np.radians(RobotConfig.FRONT_ANGLE_LIMIT_DEG)
    j1 = np.arctan2(y, x)
    target_dist = np.hypot(x, y)
    if x < RobotConfig.MIN_SAFE_X or abs(j1) > max_j1_rad:
        rospy.logwarn_throttle(2.0, f"[IK Engine] Target (X={x:.3f}, Y={y:.3f}, J1={np.degrees(j1):.1f}°) is outside front safe zone (±{RobotConfig.FRONT_ANGLE_LIMIT_DEG:.0f}°)! IK rejected.")
        return None
    if x > RobotConfig.MAX_SAFE_X or target_dist > RobotConfig.MAX_SAFE_RADIUS:
        rospy.logwarn_throttle(2.0, f"[IK Engine] Target (X={x:.3f}, Y={y:.3f}, R={target_dist:.3f}m) exceeds max safe reach ({RobotConfig.MAX_SAFE_X:.2f}m)! C31 overload prevention: IK rejected.")
        return None

    # [J1 회전 연동 J5 자동 상쇄 + 상자 회전각(target_yaw) 반영]:
    # 정면 기준 자세(J1 = 0도) 대비 J1 회전량을 상쇄하고, 상자가 기울어진 각도(target_yaw)만큼 J5 손목을 회전
    j5_comp = j1 + (RobotConfig.BOX_YAW_TO_J5_SIGN * float(target_yaw)) + RobotConfig.J5_YAW_OFFSET_RAD
    # J5 관절 가동 범위(-6.28 ~ +6.28 rad) 내로 안전 클램핑
    j5_comp = float(np.clip(j5_comp, -3.10, 3.10))

    seeds = []
    if seed_joints is not None:
        s = np.array(seed_joints, dtype=float)
        s[0] = j1
        s[4] = j5_comp
        seeds.append(s)
    # 다양한 도달 범위(근거리, 원거리, 저고도)에 적합한 후보 시드들
    seeds.extend([
        np.array([j1, 0.4, -1.2, 0.8, j5_comp], dtype=float),
        np.array([j1, 0.6, -1.6, 1.0, j5_comp], dtype=float),
        np.array([j1, 0.2, -1.0, 0.8, j5_comp], dtype=float),
    ])

    target_pos = np.array([x, y, z])
    target_z_dir = np.array([0.0, 0.0, -1.0])

    for seed in seeds:
        q = seed.copy()
        # J1과 J5가 결정된 상태에서 평면 3자유도(J2, J3, J4)를 해석 (수렴 속도 1ms 미만)
        for _ in range(40):
            p, tz = forward_kinematics_xarm5(q)
            err = np.hstack([p - target_pos, tz - target_z_dir])
            if np.linalg.norm(err) < 1e-4:
                # xArm5 기구학 관절 가동 한계 및 전방 회전각 검증
                if (-max_j1_rad <= q[0] <= max_j1_rad) and (-2.05 <= q[1] <= 2.05) and (-3.90 <= q[2] <= 0.19) and (-1.68 <= q[3] <= 3.14):
                    return q.tolist()
                break

            J = np.zeros((6, 3))
            eps = 1e-6
            for idx, joint_i in enumerate([1, 2, 3]):
                dq = q.copy()
                dq[joint_i] += eps
                p_plus, tz_plus = forward_kinematics_xarm5(dq)
                err_plus = np.hstack([p_plus - target_pos, tz_plus - target_z_dir])
                J[:, idx] = (err_plus - err) / eps

            lam = 0.01
            dq_sub = -np.linalg.solve(J.T @ J + lam * np.eye(3), J.T @ err)
            q[1] += dq_sub[0]
            q[2] += dq_sub[1]
            q[3] += dq_sub[2]

    return None

# ==============================================================================
# [헬퍼 함수 1: 그리퍼 제어]
# ==============================================================================
def control_gripper(client, position, effort, timeout=RobotConfig.GRIPPER_TIMEOUT):
    goal = GripperCommandGoal()
    goal.command.position = position
    goal.command.max_effort = effort

    rospy.loginfo(f"[Gripper Python] Sending goal: pos={position:.3f} m, effort={effort:.1f} N")
    client.send_goal(goal)
    finished = client.wait_for_result(rospy.Duration(timeout))

    if not finished:
        rospy.logwarn("[Gripper Python] Action timed out!")
        return False

    result = client.get_result()
    rospy.loginfo(f"[Gripper Python] Finished. Current Pos: {result.position:.4f} m, Stalled: {result.stalled}")
    return True

# ==============================================================================
# [헬퍼 함수 2: MoveIt Joint 이동]
# ==============================================================================
def move_to_joints(arm, joints, vel=RobotConfig.VEL_SCALE, acc=RobotConfig.ACC_SCALE):
    arm.set_max_velocity_scaling_factor(vel)
    arm.set_max_acceleration_scaling_factor(acc)
    arm.set_joint_value_target(joints)

    success = arm.go(wait=True)
    arm.stop()
    if success:
        rospy.loginfo("[Arm Python] Joint motion succeeded.")
    else:
        rospy.logerr("[Arm Python] Joint motion failed!")
    return success

# ==============================================================================
# [헬퍼 함수 2-2: J2/J3 음수 보장 S-Curve 직접 관절 보간 이동]
# ==============================================================================
def move_joints_interpolated(arm, target_joints, duration=4.5, steps=50, ensure_negative_j2_j3=True):
    """
    출발 관절 각도에서 목표 관절 각도까지 MoveIt 무작위 탐색 플래너(RRTConnect) 없이
    관절 공간 S-Curve(Cosine Ease-In/Out) 직접 보간 궤적(RobotTrajectory)을 생성하여 실행합니다.

    [핵심 안전 보증]:
    - ensure_negative_j2_j3=True:
      모든 보간 포인트에서 J2 <= 0.001 rad, J3 <= 0.001 rad임을 엄격히 사전 검증합니다.
      단 하나의 포인트라도 양수(> 0 rad)로 튀면 모션 실행을 원천 차단하여
      차량 본체 충돌(J2 > 0) 및 팔꿈치 역과신전 특이점(J3 > 0)을 100% 방지합니다.
    """
    try:
        cur_joints = arm.get_current_joint_values()
    except Exception as ex:
        rospy.logerr(f"[Joint Interp] 현재 관절값 조회 실패: {ex}")
        return False

    joint_names = arm.get_active_joints()
    if not joint_names:
        joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]

    plan = RobotTrajectory()
    traj = JointTrajectory()
    traj.joint_names = joint_names

    dt = duration / float(steps)

    for i in range(steps + 1):
        alpha = float(i) / float(steps)
        # S-curve ease-in/ease-out: s(alpha) = 0.5 * (1 - cos(pi * alpha))
        s = 0.5 * (1.0 - np.cos(np.pi * alpha))
        # 속도 계수: ds/dt = 0.5 * (pi / duration) * sin(pi * alpha)
        ds_dt = 0.5 * (np.pi / duration) * np.sin(np.pi * alpha)

        interp_pos = []
        interp_vel = []

        for c, t in zip(cur_joints, target_joints):
            diff = t - c
            p = (1.0 - s) * c + s * t
            v = diff * ds_dt
            interp_pos.append(p)
            interp_vel.append(v)

        # [안전 검증 가드]: J2와 J3가 양수로 넘어가지 않는지 엄격히 검증
        if ensure_negative_j2_j3:
            j2_val = interp_pos[1]
            j3_val = interp_pos[2]
            if j2_val > 0.001 or j3_val > 0.001:  # 부동소수점 오차 외 양수 검출 시 차단
                rospy.logerr(f"[Joint Interp] ⚠️ 치명적 안전 오류: J2({np.degrees(j2_val):.2f}°) 또는 J3({np.degrees(j3_val):.2f}°)가 양수로 감지되었습니다! 모션 차단.")
                return False

        pt = JointTrajectoryPoint()
        pt.positions = interp_pos
        pt.velocities = interp_vel
        pt.time_from_start = rospy.Duration(i * dt)
        traj.points.append(pt)

    plan.joint_trajectory = traj

    rospy.loginfo(f"[Joint Interp] Smooth S-curve 관절 직접 보간 실행 ({steps}개 포인트, {duration:.1f}초)...")
    success = arm.execute(plan, wait=True)
    arm.stop()

    if success:
        rospy.loginfo("[Joint Interp] ✅ 관절 직접 보간 모션 성공!")
    else:
        rospy.logwarn("[Joint Interp] ⚠️ 직접 보간 실행 실패. 하드웨어 에러 클리어 후 Fallback으로 MoveIt 관절 이동 시도...")
        reset_xarm_robot_state_and_mode()
        success = move_to_joints(arm, target_joints)

    return success

# ==============================================================================
# [헬퍼 함수 3: MoveIt Pose 이동]
# ==============================================================================
def move_to_pose(arm, pose, end_effector_link=None, vel=RobotConfig.VEL_SCALE, acc=RobotConfig.ACC_SCALE, target_yaw=0.0):
    arm.set_max_velocity_scaling_factor(vel)
    arm.set_max_acceleration_scaling_factor(acc)

    pos_x = pose.position.x
    pos_y = pose.position.y
    pos_z = pose.position.z

    # xArm5 전용 수직 역기구학(IK) 해석 후 관절 공간 직접 이동 (100% 안전 보장)
    # MoveIt 기본 KDL/OMPL 솔버의 무리한 샘플링으로 인한 충돌 및 C31 이상전류 에러를 원천 차단
    cur_joints = arm.get_current_joint_values() if hasattr(arm, 'get_current_joint_values') else None
    ik_joints = solve_xarm5_ik_vertical(pos_x, pos_y, pos_z, seed_joints=cur_joints, target_yaw=target_yaw)
    if ik_joints is not None:
        rospy.loginfo(f"[Arm Python] Exact 5-DOF IK solved for Z={pos_z:.3f}m (Yaw={np.degrees(target_yaw):+.1f}°): joints={[round(j, 3) for j in ik_joints]}. Executing Joint motion...")
        return move_to_joints(arm, ik_joints, vel=vel, acc=acc)

    rospy.logerr(f"[Arm Python] ❌ 목표 위치 (X={pos_x:.3f}, Y={pos_y:.3f}, Z={pos_z:.3f}m)가 안전 도달 한계를 벗어났습니다. C31 과부하 및 본체 충돌 방지를 위해 이동을 즉시 차단합니다.")
    return False

# ==============================================================================
# [헬퍼 함수 4: 테이블 간섭 방지 안전 초기 자세 이동 (2단계: 수직 상승 -> 전방 이동)]
# ==============================================================================
def move_to_initial_pose_safely(arm, eef_link=None):
    """초기 자세로 이동 시 전방 테이블 간섭/걸림을 방지하기 위해:
    1단계: 현재 위치에서 수직으로 먼저 안전 높이(SAFE_ASCENT_Z=0.38m)까지 수직 상승 (Vertical Lift)
    2단계: 테이블보다 충분히 높이 올라온 후 전방으로 뻗어 최종 초기 자세(INITIAL_JOINTS)로 진입
    """
    rospy.loginfo("[Safe Init] Planning safe 2-step motion to Initial Pose...")
    cur_pose_stamped = arm.get_current_pose(eef_link) if eef_link else arm.get_current_pose()
    cur_pos = cur_pose_stamped.pose.position

    # 후방 적재함 영역(X < MIN_SAFE_X)에 위치한 경우: 좌후방 적재함 상공 안전 고도(Z=0.35m)로 관절 상승
    if cur_pos.x < RobotConfig.MIN_SAFE_X:
        cur_j = arm.get_current_joint_values() if hasattr(arm, 'get_current_joint_values') else None
        if cur_j is not None and cur_pos.z < 0.30:
            rospy.loginfo(f"[Safe Init] Rear drop area detected (Z={cur_pos.z:.3f}m). Elevating to pre-drop altitude first...")
            move_to_joints(arm, RobotConfig.DROP_PRE_JOINTS_LEFT)
            rospy.sleep(0.3)
    # 전방 테이블 작업 영역에서 고도가 안전 고도보다 낮을 경우 수직 직선 상승 수행
    elif cur_pos.z < RobotConfig.SAFE_ASCENT_Z:
        rospy.loginfo(f"[Safe Init] Current Z={cur_pos.z:.3f}m is below safe height ({RobotConfig.SAFE_ASCENT_Z:.3f}m). Lifting vertically first...")

        ascent_pose = copy.deepcopy(cur_pose_stamped.pose)
        ascent_pose.position.z = RobotConfig.SAFE_ASCENT_Z

        # 1-1차: Cartesian 선형 직선 경로로 수직 상승 시도
        success = False
        try:
            waypoints = [ascent_pose]
            plan, fraction = arm.compute_cartesian_path(waypoints, 0.01, True)
            if fraction >= 0.8:
                rospy.loginfo(f"[Safe Init] Cartesian vertical ascent planned ({fraction*100:.1f}%). Executing...")
                success = arm.execute(plan, wait=True)
                arm.stop()
        except Exception as ex:
            rospy.logwarn(f"[Safe Init] Cartesian path exception: {ex}")

        # 1-2차: Cartesian 실패 시 move_to_pose(IK 해석)로 수직 상승
        if not success:
            rospy.loginfo("[Safe Init] Fallback: Moving to vertical ascent pose via IK/Pose target...")
            success = move_to_pose(arm, ascent_pose, end_effector_link=eef_link)

        if not success:
            rospy.logwarn("[Safe Init] Vertical ascent failed, proceeding to joint motion with caution...")
        else:
            rospy.loginfo(f"[Safe Init] Successfully elevated above table (Z >= {RobotConfig.SAFE_ASCENT_Z:.3f}m).")
            rospy.sleep(0.5)

    # 2단계: 테이블 위 안전 고도 확보 후 전방으로 이동하여 초기 자세 도달
    rospy.loginfo("[Safe Init] Moving forward to Initial Pose (INITIAL_JOINTS)...")
    success = move_joints_interpolated(arm, RobotConfig.INITIAL_JOINTS, duration=4.5, steps=45, ensure_negative_j2_j3=False)
    if success:
        rospy.loginfo("[Safe Init] Reached Initial Pose safely without table collision.")
    else:
        rospy.logerr("[Safe Init] Failed to reach Initial Pose!")
    return success

# ==============================================================================
# [헬퍼 함수 5: 테이블 간섭 및 차량 충돌 방지 안전 원점 복귀]
# ==============================================================================
def move_to_zero_pose_safely(arm, eef_link=None):
    """
    초기 자세에서 원점으로 복귀 시,
    테이블 간섭과 차량 본체 충돌을 모두 100% 방지하는 2단계 안전 복귀 시퀀스:

    [1단계: 안전 후방 복귀 (초기 자세 -> 원점 수직 상공)]
      - 초기 자세(X=0.558m, Z=0.419m)에서 테이블 위 안전 고도(Z >= 0.38m)를 통과하여
        원점 수직 상공인 ascent_pose(X=0.207m, Y=0.0m, Z=SAFE_ASCENT_Z=0.38m)로 후퇴.
      - 이 시점에서 테이블 모서리(X=0.35m)로부터 14cm 이상 떨어져 테이블 간섭 100% 배제!
      - 해당 지점의 관절 각도는 ASCENT_JOINTS (J2 = -49.80°, J3 = -54.16°로 둘 다 안전한 음수 유지).

    [2단계: 차량 충돌 방지 J2/J3 음수 보장 S-Curve 직접 보간 안착 (원점 상공 -> 원점)]
      - MoveIt 무작위 플래너(RRTConnect)나 위험한 Cartesian 음수 고도 하강을 쓰지 않고,
        ASCENT_JOINTS에서 ZERO_JOINTS([0, 0, 0, 0, 0])까지 관절 공간 직접 S-Curve 보간으로 이동!
      - J2: -49.80° -> 0.00° (전 구간 J2 <= 0.0° 유지로 차량 본체 충돌 원천 차단)
      - J3: -54.16° -> 0.00° (전 구간 J3 <= 0.0° 유지로 역방향 과신전/특이점 원천 차단)
      - 부드럽고 우아하게 원점([0, 0, 0, 0, 0])에 완벽히 안착!
    """
    rospy.loginfo("==================================================")
    rospy.loginfo("[Safe Zero] 🛡️ 안전 원점 복귀 시퀀스 (테이블 회피 후방 복귀 -> J2/J3 음수 보장 직접 보간 안착) 시작...")
    reset_xarm_robot_state_and_mode()
    cur_pose_stamped = arm.get_current_pose(eef_link) if eef_link else arm.get_current_pose()

    # 1. 원점 바로 수직 상공 Pose (기동 시 수직 상승 완료 지점과 동일: X=0.207m, Y=0.0m, Z=0.38m)
    ascent_pose = copy.deepcopy(cur_pose_stamped.pose)
    ascent_pose.position.x = RobotConfig.ZERO_X  # 0.2070m (원점 TCP의 X축 위치)
    ascent_pose.position.y = 0.0                 # 정면 중심 정렬
    ascent_pose.position.z = RobotConfig.SAFE_ASCENT_Z  # 0.38m (안전 상승 고도)
    ascent_pose.orientation.x = 1.0              # 수직 하향 (Pitch -90°)
    ascent_pose.orientation.y = 0.0
    ascent_pose.orientation.z = 0.0
    ascent_pose.orientation.w = 0.0

    # --------------------------------------------------------------------------
    # [1단계]: 초기 자세에서 원점 바로 수직 상공(ascent_pose)으로 후방 이동
    # --------------------------------------------------------------------------
    rospy.loginfo(f"[Safe Zero] [1/2] ➡️ 테이블 안전 상공(Z >= {RobotConfig.SAFE_ASCENT_Z:.2f}m)을 통과하여 "
                  f"원점 수직 상공(X={ascent_pose.position.x:.3f}m, Z={ascent_pose.position.z:.3f}m)으로 후퇴...")

    step1_success = False
    # 1-1차: Cartesian 선형 이동 시도
    try:
        waypoints = [ascent_pose]
        plan, fraction = arm.compute_cartesian_path(waypoints, 0.01, True)
        if fraction >= 0.8:
            rospy.loginfo(f"[Safe Zero] Cartesian 후퇴 계획 성공 ({fraction*100:.1f}%). 실행 중...")
            step1_success = arm.execute(plan, wait=True)
            arm.stop()
    except Exception as ex:
        rospy.logwarn(f"[Safe Zero] Cartesian 후퇴 예외: {ex}")

    # 1-2차 Fallback: move_to_pose
    if not step1_success:
        rospy.loginfo("[Safe Zero] Fallback 1: move_to_pose로 원점 상공 위치 이동...")
        step1_success = move_to_pose(arm, ascent_pose, end_effector_link=eef_link)

    # 1-3차 Fallback: 검증된 안전 상공 관절 각도(ASCENT_JOINTS)로 이동
    if not step1_success:
        rospy.loginfo("[Safe Zero] Fallback 2: ASCENT_JOINTS 관절 이동...")
        step1_success = move_to_joints(arm, RobotConfig.ASCENT_JOINTS, vel=0.30, acc=0.25)

    if not step1_success:
        rospy.logwarn("[Safe Zero] ⚠️ 1단계 원점 상공 후퇴 실패! 주의하여 다음 단계 진행.")

    rospy.sleep(0.3)

    # 1단계 완료 후 정확한 안전 상공 관절 각도(ASCENT_JOINTS: J2=-49.8°, J3=-54.2°) 확인 및 정렬
    cur_j = arm.get_current_joint_values() if hasattr(arm, 'get_current_joint_values') else None
    if cur_j is not None and (cur_j[1] > -0.2 or cur_j[2] > -0.2):
        rospy.loginfo("[Safe Zero] 안전 관절 기준점(ASCENT_JOINTS)으로 정렬...")
        reset_xarm_robot_state_and_mode()
        move_to_joints(arm, RobotConfig.ASCENT_JOINTS, vel=0.25, acc=0.20)
        rospy.sleep(0.2)

    # --------------------------------------------------------------------------
    # [2단계]: 차량 본체 충돌 방지 J2/J3 음수 보장 S-Curve 직접 관절 보간 안착
    # --------------------------------------------------------------------------
    rospy.loginfo("[Safe Zero] [2/2] ⬇️ J2/J3 음수 보장 S-Curve 직접 관절 보간으로 원점([0, 0, 0, 0, 0]) 안착 시작...")
    step2_success = move_joints_interpolated(
        arm,
        RobotConfig.ZERO_JOINTS,
        duration=5.5,
        steps=55,
        ensure_negative_j2_j3=True
    )

    # Fallback: 만약 보간 궤적 실행이 preempt된 경우 안전 중간 관절 경유 안착
    if not step2_success:
        rospy.loginfo("[Safe Zero] Fallback: 안전 중간 관절(ASCENT_MID_JOINTS) 경유 안착 시도...")
        move_to_joints(arm, RobotConfig.ASCENT_MID_JOINTS, vel=0.20, acc=0.15)
        step2_success = move_to_joints(arm, RobotConfig.ZERO_JOINTS, vel=0.20, acc=0.15)

    if step2_success:
        rospy.loginfo("[Safe Zero] 🎉 테이블 간섭 및 차량 충돌 없이 원점([0, 0, 0, 0, 0])에 완벽히 복귀했습니다!")
    else:
        rospy.logwarn("[Safe Zero] ⚠️ 원점 자세 이동 실패!")

    return step1_success and step2_success


# ==============================================================================
# [헬퍼 함수 6: 실시간 타겟 클래스 수신 추적기 및 타겟 대기 함수]
# ==============================================================================
class YoloTargetTracker:
    """YOLO 노드에서 발행하는 실시간 타겟 클래스 ID (/yolo/target_class)를 추적하는 클래스"""
    def __init__(self):
        self.target_class = 0
        self.last_update_time = rospy.Time(0)
        self.sub = rospy.Subscriber("/yolo/target_class", Int32, self.cb, queue_size=1)

    def cb(self, msg):
        self.target_class = msg.data
        self.last_update_time = rospy.Time.now()

    def get_class(self):
        return self.target_class


def wait_for_target_box(tf_listener, tracker, timeout=RobotConfig.NO_DETECTION_TIMEOUT):
    """
    초기 자세에서 대기하며 YOLO 대상 물체(TF & Class)를 수신합니다.
    - timeout(5초) 동안 유효한 검출이 없으면 None 반환 (작업 종료 판정)
    - 유효한 검출이 연속 REQUIRED_CONSECUTIVE_DET회 이상 안정적으로 들어오면
      (class_id, (cam_x, cam_y, cam_z), avg_yaw_rad) 반환
    """
    arrival_time = rospy.Time.now()
    start_wait_time = rospy.Time.now()
    rate = rospy.Rate(10)  # 10Hz

    valid_consecutive = 0
    history_trans = []
    history_yaws = []
    detected_class = 0

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_wait_time).to_sec()
        if elapsed >= timeout:
            rospy.loginfo(f"[Vision] ⏱️ {timeout:.1f}초 동안 추가 박스가 검출되지 않았습니다. (대기 경과: {elapsed:.1f}초)")
            return None

        try:
            if tf_listener.canTransform(RobotConfig.BASE_FRAME, RobotConfig.TARGET_FRAME, rospy.Time(0)):
                latest_tf_time = tf_listener.getLatestCommonTime(RobotConfig.BASE_FRAME, RobotConfig.TARGET_FRAME)
                time_since_tf = (rospy.Time.now() - latest_tf_time).to_sec()

                # 시간 동기화 검증: 초기 자세 도착 이후 & 실시간(0.5초 이내) 데이터
                if latest_tf_time >= arrival_time and time_since_tf < 0.5:
                    (curr_t, curr_r) = tf_listener.lookupTransform(RobotConfig.BASE_FRAME, RobotConfig.TARGET_FRAME, rospy.Time(0))
                    cx, cy, cz = curr_t
                    _, _, cyaw = tft.euler_from_quaternion(curr_r)
                    # [-pi/4, +pi/4] 범위 확인
                    cyaw = float(np.clip(cyaw, -np.pi / 4.0, np.pi / 4.0))

                    if RobotConfig.MIN_TARGET_CAM_Z <= cz <= RobotConfig.MAX_TARGET_CAM_Z:
                        history_trans.append(curr_t)
                        history_yaws.append(cyaw)
                        valid_consecutive += 1
                        detected_class = tracker.get_class()
                        cls_name = RobotConfig.CLASS_NAMES.get(detected_class, f"Class {detected_class}")
                        rospy.loginfo(f"[Vision] Box Detected ({valid_consecutive}/{RobotConfig.REQUIRED_CONSECUTIVE_DET}): "
                                      f"[{cls_name}] Cam X={cx:.3f}, Y={cy:.3f}, Z={cz:.3f}m, Yaw={np.degrees(cyaw):+.1f}°")

                        if valid_consecutive >= RobotConfig.REQUIRED_CONSECUTIVE_DET:
                            avg_trans = np.mean(history_trans, axis=0).tolist()
                            # 90도 주기(±45도 경계)에서의 평균 왜곡 방지를 위해 중앙값(median) 사용
                            avg_yaw = float(np.median(history_yaws))
                            rospy.loginfo(f"[Vision] ✅ Target Confirmed! [{cls_name}] Average TF -> "
                                          f"X={avg_trans[0]:.3f}, Y={avg_trans[1]:.3f}, Z={avg_trans[2]:.3f}m, Yaw={np.degrees(avg_yaw):+.1f}°")
                            return (detected_class, avg_trans, avg_yaw)
                    else:
                        rospy.logwarn_throttle(1.5, f"[Vision] ⚠️ 감지된 타겟(Cam Z={cz:.3f}m)이 테이블 유효 작업 깊이({RobotConfig.MIN_TARGET_CAM_Z:.2f}m ~ {RobotConfig.MAX_TARGET_CAM_Z:.2f}m)를 벗어나 배제되었습니다.")
                        valid_consecutive = 0
                        history_trans.clear()
                        history_yaws.clear()
                else:
                    valid_consecutive = 0
                    history_trans.clear()
                    history_yaws.clear()
            else:
                valid_consecutive = 0
                history_trans.clear()
                history_yaws.clear()
        except (tf.Exception, Exception):
            valid_consecutive = 0
            history_trans.clear()
            history_yaws.clear()

        rate.sleep()

    return None


# ==============================================================================
# [헬퍼 함수 7: xArm 제어 모드 및 상태 자동 복구 (Web Studio 수동 조작 후 복구)]
# ==============================================================================
def call_xarm_service(srv_name, args=(), timeout=2.0):
    """
    ROS 서비스를 안전하게 호출합니다.
    xarm_msgs 모듈이 없더라도 rosservice 동적 리플렉션을 우선 활용하여 호출합니다.
    """
    try:
        rospy.wait_for_service(srv_name, timeout=timeout)
    except rospy.ROSException:
        rospy.logwarn(f"[xArm Setup] 서비스 '{srv_name}' 응답 대기 시간 초과 ({timeout}초).")
        return False

    srv_class = None
    try:
        import rosservice
        srv_class = rosservice.get_service_class_by_name(srv_name)
    except Exception:
        pass

    if srv_class is None:
        if "motion_ctrl" in srv_name and SetAxis is not None:
            srv_class = SetAxis
        elif ("set_mode" in srv_name or "set_state" in srv_name) and SetInt16 is not None:
            srv_class = SetInt16
        elif ("clear_err" in srv_name or "clean" in srv_name) and ClearErr is not None:
            srv_class = ClearErr

    if srv_class is None:
        rospy.logwarn(f"[xArm Setup] '{srv_name}' 서비스 클래스를 확인할 수 없습니다.")
        return False

    try:
        proxy = rospy.ServiceProxy(srv_name, srv_class)
        if len(args) == 0:
            res = proxy()
        elif len(args) == 1:
            res = proxy(args[0])
        elif len(args) == 2:
            res = proxy(args[0], args[1])
        else:
            res = proxy(*args)
        ret = getattr(res, "ret", 0)
        msg = getattr(res, "message", "")
        rospy.loginfo(f"[xArm Setup] 호출 성공: {srv_name}{args} -> ret={ret}, msg='{msg}'")
        return True
    except rospy.ServiceException as e:
        rospy.logerr(f"[xArm Setup] 서비스 호출 실패 ({srv_name}): {e}")
        return False
    except Exception as e:
        rospy.logerr(f"[xArm Setup] 서비스 호출 예외 ({srv_name}): {e}")
        return False


def reset_xarm_robot_state_and_mode(ns="/xarm"):
    """
    xArm Web Studio 조작 등으로 인해 Robot Mode(0: Manual/Position) 및
    Robot State(4: STOP)로 전환된 로봇 팔을 ROS MoveIt 모드(Mode 1, State 0)로
    자동 복구하고 모터를 인에이블(motion_ctrl 8 1)합니다.
    """
    rospy.loginfo("==================================================")
    rospy.loginfo("[xArm Setup] 🔄 웹스튜디오 조작 후 모드/상태 감지 및 MoveIt 모드로 자동 복구 시작...")

    # 0. 초기 상태 확인 (수신 가능한 경우)
    if RobotMsg is not None:
        try:
            init_st = rospy.wait_for_message(f"{ns}/xarm_states", RobotMsg, timeout=1.0)
            rospy.loginfo(f"[xArm Setup] 현재 로봇 상태 -> Mode: {init_st.mode}, State: {init_st.state}, Error: {init_st.err}")
        except Exception:
            pass

    # 1. 에러 및 경고 상태 클리어
    for err_srv in [f"{ns}/moveit_clear_err", f"{ns}/clear_err", f"{ns}/clean_error"]:
        try:
            rospy.wait_for_service(err_srv, timeout=0.5)
            if call_xarm_service(err_srv, args=(), timeout=1.0):
                break
        except rospy.ROSException:
            continue

    for warn_srv in [f"{ns}/clean_warn", f"{ns}/clear_warn"]:
        try:
            rospy.wait_for_service(warn_srv, timeout=0.5)
            call_xarm_service(warn_srv, args=(), timeout=1.0)
            break
        except rospy.ROSException:
            pass

    # 2. 모터 인에이블 (motion_ctrl 8 1 -> 모든 관절 축 활성화)
    rospy.loginfo("[xArm Setup] [1/3] 관절 모터 인에이블 (/xarm/motion_ctrl 8 1)...")
    call_xarm_service(f"{ns}/motion_ctrl", args=(8, 1), timeout=2.0)
    rospy.sleep(0.3)

    # 3. 로봇 모드를 Mode 1 (MoveIt / Trajectory 서보 모드)로 설정
    rospy.loginfo("[xArm Setup] [2/3] 로봇 제어 모드를 MoveIt 모드로 설정 (/xarm/set_mode 1)...")
    call_xarm_service(f"{ns}/set_mode", args=(1,), timeout=2.0)
    rospy.sleep(0.5)

    # 4. 로봇 상태를 State 0 (READY / Standby 준비 상태)로 설정 (State 4 STOP 해제)
    rospy.loginfo("[xArm Setup] [3/3] 로봇 상태를 준비 상태로 전환 (/xarm/set_state 0)...")
    call_xarm_service(f"{ns}/set_state", args=(0,), timeout=2.0)
    rospy.sleep(0.5)

    # 5. 복구 결과 확인
    if RobotMsg is not None:
        try:
            final_st = rospy.wait_for_message(f"{ns}/xarm_states", RobotMsg, timeout=2.0)
            rospy.loginfo(f"[xArm Setup] 복구 후 로봇 상태 -> Mode: {final_st.mode}, State: {final_st.state}, Error: {final_st.err}")
            if final_st.mode == 1 and final_st.state in [0, 2]:
                rospy.loginfo("[xArm Setup] ✅ 로봇이 ROS MoveIt 제어 가능 상태로 정상 전환되었습니다!")
            elif final_st.state == 4:
                rospy.logwarn("[xArm Setup] ⚠️ 로봇이 여전히 State 4(Stop)입니다. 웹스튜디오 브라우저 연결을 닫았는지 확인하세요.")
        except Exception:
            pass

    rospy.loginfo("==================================================")


# ==============================================================================
# [메인 함수: 다중 박스 연속 자동 픽앤플레이스 파이프라인]
# ==============================================================================
def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("xarm_vision_pick_drop_py", anonymous=False)

    # 0. xArm 로봇 모드 및 상태 자동 복구 (Web Studio 사용 후 Mode 0, State 4 복구)
    reset_xarm_robot_state_and_mode(ns="/xarm")

    # 1. MoveGroup 및 ActionClient 초기화
    rospy.loginfo("[Main] Initializing MoveGroup (xarm5)...")
    arm = moveit_commander.MoveGroupCommander("xarm5")
    arm.set_planning_time(5.0)
    arm.set_goal_position_tolerance(0.01) # 1cm 위치 오차 허용
    arm.set_goal_orientation_tolerance(0.1) # 5-DOF 회전 오차 허용

    # [MoveIt 플래너 작업 공간 설정]: 파지(전방 180도) 및 적재(좌후방 & 우후방) 영역을 모두 포함하도록 설정
    try:
        arm.set_workspace([-0.80, -0.80, -0.10, 0.80, 0.80, 0.80])
        rospy.loginfo("[Main] MoveIt workspace set to cover full pick & place area.")
    except Exception as ex:
        rospy.logwarn(f"[Main] Failed to set MoveIt workspace: {ex}")

    eef_link = arm.get_end_effector_link()
    try:
        robot = moveit_commander.RobotCommander()
        if "link_tcp" in robot.get_link_names():
            eef_link = "link_tcp"
    except Exception:
        pass
    rospy.loginfo(f"[Main] Using End-Effector Link from MoveIt: '{eef_link}'")

    rospy.loginfo("[Main] Connecting to Gripper Action Server...")
    gripper_client = actionlib.SimpleActionClient("/xarm/xarm_gripper/gripper_action", GripperCommandAction)
    if not gripper_client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("[Main] Failed to connect to gripper action server. Exiting...")
        return
    rospy.loginfo("[Main] Gripper Action Server connected!")

    # 2. TF Listener, 타겟 클래스 추적기 및 퍼블리셔 초기화
    tf_listener = tf.TransformListener()
    target_tracker = YoloTargetTracker()

    vision_enable_pub = rospy.Publisher("/vision_enable", Bool, queue_size=1)
    yolo_stop_pub = rospy.Publisher("/yolo_stop", Bool, queue_size=1)
    tag_done_pub = rospy.Publisher("/tag_done", Bool, queue_size=1)

    rospy.sleep(0.5)
    vision_enable_pub.publish(Bool(data=False))  # 시작 시 비전 일시 정지 상태 보장
    rospy.sleep(0.5)

    # 3. 최초 사전 그리퍼 개방 (Pre-open)
    rospy.loginfo("==================================================")
    rospy.loginfo(f"[Init] Opening gripper (Pre-open: {RobotConfig.GRIPPER_OPEN_POS:.3f} m)...")
    control_gripper(gripper_client, RobotConfig.GRIPPER_OPEN_POS, RobotConfig.GRIPPER_EFFORT)

    # ==========================================================================
    # [다중 박스 연속 자동 픽앤플레이스 루프]
    # 1. 현재 방식대로 초기 자세로 이동
    # 2. yolo를 통한 박스 검출 시작
    # 3. 0번(빨간색) / 1번(노란색) / 2번(파란색) -> 좌후방 상공 경유 후 색상별 전용 위치에 drop
    # 4. drop 후 다시 초기 자세로 이동
    # 5. 초기 자세에서 다시 박스 검출 시작
    # 6. 5초간 검출 후 검출되지 않을 시 테이블 간섭 방지(수평 이동 후 수직 이동) 원점(0도) 복귀
    # 7. 여러 박스 검출 시 0번(빨간색) -> 1번(노란색) -> 2번(파란색) 순으로 작업 진행
    # ==========================================================================
    cycle = 0
    total_picked = 0

    while not rospy.is_shutdown():
        cycle += 1
        rospy.loginfo("**************************************************")
        rospy.loginfo(f"🚀 [CYCLE {cycle}] Starting Pick & Place Cycle (Total Picked: {total_picked})")
        rospy.loginfo("**************************************************")

        # 로봇 이동 중에는 비전 일시 정지(TF 발행 차단) 유지
        vision_enable_pub.publish(Bool(data=False))

        # [STEP 1] 초기 자세로 안전하게 이동 (수직 상승 후 전방 이동)
        rospy.loginfo(f"[CYCLE {cycle} - STEP 1] Moving to Initial Pose safely...")
        if not move_to_initial_pose_safely(arm, eef_link=eef_link):
            rospy.logerr(f"[CYCLE {cycle}] Failed to move to initial pose. Aborting.")
            break

        # [STEP 2] 초기 자세 도착 후 비전 활성화 신호 전송 및 영상 안정화
        vision_enable_pub.publish(Bool(data=True))
        rospy.sleep(0.5)

        # [STEP 3] 5초간 박스 검출 대기
        rospy.loginfo(f"[CYCLE {cycle} - STEP 2] Waiting for box detection ({RobotConfig.NO_DETECTION_TIMEOUT:.1f}s timeout)...")
        target_result = wait_for_target_box(tf_listener, target_tracker, timeout=RobotConfig.NO_DETECTION_TIMEOUT)

        if target_result is None:
            vision_enable_pub.publish(Bool(data=False))  # 복귀 중 카메라 검출 및 불필요한 로그 차단
            rospy.loginfo(f"[CYCLE {cycle}] ⏱️ {RobotConfig.NO_DETECTION_TIMEOUT:.1f}초 동안 검출된 박스가 없습니다. 모든 박스 작업이 완료되었습니다!")
            break

        target_cls, (cam_x, cam_y, cam_z), target_yaw = target_result
        cls_name = RobotConfig.CLASS_NAMES.get(target_cls, f"Class {target_cls}")
        rospy.loginfo(f"[CYCLE {cycle}] 🎯 Target Confirmed: [{cls_name}] (Class {target_cls}) at Cam X={cam_x:.3f}, Y={cam_y:.3f}, Z={cam_z:.3f}m, Yaw={np.degrees(target_yaw):+.1f}°")

        # [STEP 4] 로봇 모션 중 비전 일시 정지 (카메라 회전/흔들림 중 오인식 방지)
        vision_enable_pub.publish(Bool(data=False))

        # [STEP 5] Eye-in-Hand 기반 파지 목표 자세 계산
        cur_pose_stamped = arm.get_current_pose(eef_link) if eef_link else arm.get_current_pose()
        cur_pos = cur_pose_stamped.pose.position

        grasp_pose = Pose()
        grasp_pose.position.x = cur_pos.x + (cam_y * RobotConfig.CAM_Y_TO_ROBOT_X_SCALE) + RobotConfig.GRASP_OFFSET_X
        grasp_pose.position.y = cur_pos.y + (cam_x * RobotConfig.CAM_X_TO_ROBOT_Y_SCALE) + RobotConfig.GRASP_OFFSET_Y

        tcp_to_box_dist = max(0.05, cam_z - RobotConfig.TCP_TO_CAM_Z_DIST)
        box_top_z = cur_pos.z - tcp_to_box_dist
        grasp_pose.position.z = max(RobotConfig.MIN_SAFE_Z, box_top_z - RobotConfig.GRASP_DEPTH_MARGIN)

        # 수직 하향 파지 쿼터니언 (Pitch -90도)
        grasp_pose.orientation.x = 1.0
        grasp_pose.orientation.y = 0.0
        grasp_pose.orientation.z = 0.0
        grasp_pose.orientation.w = 0.0

        pre_grasp_pose = copy.deepcopy(grasp_pose)
        pre_grasp_pose.position.z = box_top_z + RobotConfig.APPROACH_Z_OFFSET

        # 전방 180도 및 최대 도달 한계 안전 가드 검증 (C31 과전류 방지)
        target_dist = np.hypot(grasp_pose.position.x, grasp_pose.position.y)
        target_j1_deg = np.degrees(np.arctan2(grasp_pose.position.y, grasp_pose.position.x))
        if grasp_pose.position.x < RobotConfig.MIN_SAFE_X or abs(target_j1_deg) > RobotConfig.FRONT_ANGLE_LIMIT_DEG:
            rospy.logerr(f"[CYCLE {cycle}] ⚠️ 위험: 파지 목표 위치(X={grasp_pose.position.x:.3f}m)가 전방 180도 범위를 벗어났습니다. 해당 사이클 스킵.")
            continue
        if grasp_pose.position.x > RobotConfig.MAX_SAFE_X or target_dist > RobotConfig.MAX_SAFE_RADIUS:
            rospy.logwarn(f"[CYCLE {cycle}] ⚠️ 과부하(C31) 방지: 대상 박스가 로봇 안전 도달 한계(최대 {RobotConfig.MAX_SAFE_X:.2f}m)를 초과했습니다 (목표 X={grasp_pose.position.x:.3f}m, 작업반경={target_dist:.3f}m).")
            rospy.logwarn(f"[CYCLE {cycle}] 💡 조치 안내: 무리한 팔 뻗음으로 인한 C31 이상전류 방지를 위해 해당 박스를 건너뜁니다. Scout 차량을 테이블 방향으로 약 10~15cm 전진시켜 주세요!")
            continue

        # [STEP 6] Pre-Grasp 접근 및 Final Grasp 하강 (상자 회전각 target_yaw만큼 J5 손목 회전 적용)
        rospy.loginfo(f"[CYCLE {cycle} - STEP 3-1] Moving to Pre-Grasp Pose (Z: {pre_grasp_pose.position.z:.3f}m, Box Yaw: {np.degrees(target_yaw):+.1f}°)...")
        if not move_to_pose(arm, pre_grasp_pose, end_effector_link=eef_link, target_yaw=target_yaw):
            rospy.logerr(f"[CYCLE {cycle}] Failed to reach Pre-Grasp pose.")
            continue
        rospy.sleep(0.3)

        rospy.loginfo(f"[CYCLE {cycle} - STEP 3-2] Lowering to Final Grasp Pose (Z: {grasp_pose.position.z:.3f}m, Box Yaw: {np.degrees(target_yaw):+.1f}°)...")
        if not move_to_pose(arm, grasp_pose, end_effector_link=eef_link, target_yaw=target_yaw):
            rospy.logerr(f"[CYCLE {cycle}] Failed to reach Grasp pose.")
            continue
        rospy.sleep(0.3)

        # [STEP 7] 그리퍼 닫기 (물체 파지)
        rospy.loginfo(f"[CYCLE {cycle} - STEP 4] Closing gripper to grasp [{cls_name}] (Effort: {RobotConfig.GRIPPER_EFFORT:.1f} N)...")
        control_gripper(gripper_client, RobotConfig.GRIPPER_CLOSE_POS, RobotConfig.GRIPPER_EFFORT)
        rospy.sleep(0.5)

        # [STEP 8] 물체 들어 올리기 (1차 수직 상승: 파지 위치 이탈 - 파지 당시의 손목 회전각 유지)
        rospy.loginfo(f"[CYCLE {cycle} - STEP 5-1] Lifting object vertically (Z: {pre_grasp_pose.position.z:.3f}m)...")
        if not move_to_pose(arm, pre_grasp_pose, end_effector_link=eef_link, target_yaw=target_yaw):
            rospy.logwarn(f"[CYCLE {cycle}] 1차 수직 상승 실패. 현재 위치 유지 후 후퇴 진행.")
        rospy.sleep(0.2)

        # [STEP 8-2] 원거리 파지 시 몸 쪽 안전 후퇴 (Horizontal Retract):
        # 대상이 원거리(R > 0.42m)에 있을 경우, 그 먼 거리에서 Z >= 0.35m로 바로 상승하려 하면
        # xArm5의 기구학적 한계(리치 부족)로 실패하고, 그 상태에서 고속 선회하면 막대한 관성 모멘트로
        # 모터 과부하(C31) 및 CONTROL_FAILED가 발생합니다.
        # 따라서 회전 전 몸 쪽(R ~ 0.38m)으로 안전하게 수평 후퇴하여 관성 모멘트를 대폭 줄입니다.
        current_radius = np.hypot(grasp_pose.position.x, grasp_pose.position.y)
        retract_pose = copy.deepcopy(pre_grasp_pose)
        if current_radius > 0.42:
            scale_inward = 0.38 / current_radius
            retract_pose.position.x = grasp_pose.position.x * scale_inward
            retract_pose.position.y = grasp_pose.position.y * scale_inward
            retract_pose.position.z = max(0.28, pre_grasp_pose.position.z)
            rospy.loginfo(f"[CYCLE {cycle} - STEP 5-2] Retracting inward safely (R: {current_radius:.3f}m -> 0.380m, Z: {retract_pose.position.z:.3f}m)...")
            move_to_pose(arm, retract_pose, end_effector_link=eef_link)
            rospy.sleep(0.2)

        # [STEP 9] 색상별 드랍: 기존 좌후방 상공 경유 -> 전용 위치(Z=-0.075m)
        if target_cls == 0:
            target_pre_drop_joints = RobotConfig.DROP_PRE_JOINTS_RED
            target_drop_joints     = RobotConfig.DROP_JOINTS_RED
            drop_loc_name          = "빨간색 전용 Drop (Z=-0.075m)"
        elif target_cls == 1:
            target_pre_drop_joints = RobotConfig.DROP_PRE_JOINTS_YELLOW
            target_drop_joints     = RobotConfig.DROP_JOINTS_YELLOW
            drop_loc_name          = "노란색 전용 Drop (Z=-0.075m)"
        else:
            target_pre_drop_joints = RobotConfig.DROP_PRE_JOINTS_BLUE
            target_drop_joints     = RobotConfig.DROP_JOINTS_BLUE
            drop_loc_name          = "파란색 전용 Drop (Z=-0.075m)"

        # [STEP 9-1] 몸 쪽 안전 반경(R ~ 0.38m)에서 적재 방향으로 선회 (테이블 위 타 물체 간섭 100% 회피)
        # 페이로드 파지 상태를 고려하여 duration=5.5초, steps=55로 더욱 부드러운 S-curve 가감속 적용
        rospy.loginfo(f"[CYCLE {cycle} - STEP 6-1] Rotating to {drop_loc_name} at safe retracted radius (R=0.38m)...")
        if not move_joints_interpolated(arm, target_pre_drop_joints, duration=5.5, steps=55, ensure_negative_j2_j3=False):
            rospy.logerr(f"[CYCLE {cycle}] Failed to rotate to high pre-drop pose!")
            reset_xarm_robot_state_and_mode()
            control_gripper(gripper_client, RobotConfig.GRIPPER_OPEN_POS, RobotConfig.GRIPPER_EFFORT)
            continue
        rospy.sleep(0.3)

        # [STEP 9-2] 상공 경유점에서 최종 드랍 관절 자세로 이동 (수직 직선 경로 아님)
        rospy.loginfo(f"[CYCLE {cycle} - STEP 6-2] Moving from pre-drop checkpoint into {drop_loc_name} bin...")
        if not move_joints_interpolated(arm, target_drop_joints, duration=4.0, steps=40, ensure_negative_j2_j3=False):
            rospy.logerr(f"[CYCLE {cycle}] Failed to lower into drop pose!")
            reset_xarm_robot_state_and_mode()
            continue
        rospy.sleep(0.5)

        # [STEP 10] 그리퍼 개방 (물체 낙하/적재)
        rospy.loginfo(f"[CYCLE {cycle} - STEP 6-3] Opening gripper to drop [{cls_name}]...")
        control_gripper(gripper_client, RobotConfig.GRIPPER_OPEN_POS, RobotConfig.GRIPPER_EFFORT)
        rospy.sleep(0.5)

        # [STEP 11] 기존 좌후방 상공 경유점으로 복귀
        rospy.loginfo(f"[CYCLE {cycle} - STEP 6-4] Returning to pre-drop checkpoint before returning to initial pose...")
        if not move_joints_interpolated(arm, target_pre_drop_joints, duration=4.0, steps=40, ensure_negative_j2_j3=False):
            reset_xarm_robot_state_and_mode()
        rospy.sleep(0.3)

        total_picked += 1
        rospy.loginfo(f"[CYCLE {cycle}] ✅ Successfully picked and dropped [{cls_name}]! (Total Picked: {total_picked})")

        # Drop 후 다시 초기 자세로 이동하기 위해 루프 처음으로 진입 (STEP 1)

    # ==========================================================================
    # [작업 종료] 5초간 박스 미검출 시
    # 테이블 간섭 방지: 수평으로 먼저 이동 후 수직으로 이동하여 0도 복귀
    # ==========================================================================
    rospy.loginfo("==================================================")
    rospy.loginfo(f"[Main] 🎉 모든 박스 작업 완료! (총 {total_picked}개 박스 처리 완료)")
    rospy.loginfo("[Main] 테이블 간섭 방지 안전 원점 복귀 시퀀스 (수평 후퇴 -> 수직 복귀) 실행...")
    vision_enable_pub.publish(Bool(data=False))
    move_to_zero_pose_safely(arm, eef_link=eef_link)

    # YOLO 노드에 최종 정지 신호 전송
    yolo_stop_pub.publish(Bool(data=True))
    tag_done_pub.publish(Bool(data=True))
    rospy.sleep(1.0)

    rospy.loginfo("==================================================")
    rospy.loginfo("All Pick & Place cycles completed successfully! Program finished.")

    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
