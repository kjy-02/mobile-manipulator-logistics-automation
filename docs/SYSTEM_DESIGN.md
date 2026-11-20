# 시스템 기술 상세

SCOUT 2.0이 픽업존까지 이동한 후 멈추면 xArm5가 RGB-D 카메라로 검출한 상자를 차량 측면 트레이에 적재합니다. [실내 통합 시연](https://youtu.be/7SqIaekgKKs)에서 빨강·노랑·파랑 상자의 순차 적재를 확인했습니다.

## 지도 작성과 위치 추정

[지도 작성 런치](../launch/mapping.launch)는 VLP-16의 점군을 전처리하고 FAST_GICP로 스캔 간 움직임을 추정합니다. HDL Graph SLAM이 키프레임과 루프 제약을 자세 그래프에 반영하고 결과 지도를 PCD로 저장합니다. 시연은 [실내 지도](../maps/m304.pcd)를 사용합니다.

[위치 추정 런치](../local_ws/src/scout_dwa/launch/hdl_localization_map.launch)는 HDL Localization의 NDT_OMP로 실시간 점군을 저장 지도에 정합합니다. RViz에서 초기 위치와 픽업존 목표를 지정합니다. DWA는 차량의 측정 속도를 `/scout/odom`에서 받고, 위치 추정에 사용되는 odometry는 `/localization/odom`으로 분리합니다.

## DWA 접근과 단계 전환

[점군 어댑터](../local_ws/src/scout_dwa/src/cloud_obstacle_adapter.cpp)가 실시간 LiDAR 점군을 장애물 격자로 변환합니다. [DWA](../local_ws/src/dwa_planner/src/dwa_planner.cpp)는 현재 속도에서 도달할 수 있는 속도 후보로 궤적을 예측하고, 충돌 검사와 목표·장애물·속도 비용으로 명령을 고릅니다. 정지 상태에서 가속도 제한으로 최대 후보가 설정된 최소 속도보다 낮아지면 최소 후보를 최대 후보와 같은 값으로 제한합니다. 시연은 목표점에 국소적으로 접근하는 방식입니다.

[실내 설정](../local_ws/src/scout_dwa/config/dwa_scout_low_speed_test.yaml)의 목표 속도는 0.55 m/s, 최소 속도는 0.1 m/s, 예측 시간은 3 s, 도착 허용 거리는 0.3 m입니다. 목표점 중심 0.75 m 안의 점을 DWA 장애물 후보에서 제외하도록 설정했습니다. 이 수치는 소프트웨어 설정값입니다.

| 실행 단계 | 역할 |
| --- | --- |
| [`start_all.sh`](../start_all.sh) | 패키지를 빌드하고 [통합 런치](../launch/system.launch)로 차량·센서·팔을 준비 |
| [`run_goal_navigation.py`](../local_ws/src/scout_dwa/scripts/run_goal_navigation.py) | 목표를 받고 [DWA 런치](../local_ws/src/scout_dwa/launch/low_speed_planner.launch)를 구동. `finish_flag`를 받으면 정지 명령을 보내고 DWA 종료 |
| [`command_watchdog.py`](../local_ws/src/scout_dwa/scripts/command_watchdog.py) | 주행 프로그램의 차량 명령을 `/cmd_vel`로 전달. 입력이 끊기면 정지 명령 발행 |
| [`run_dwa_then_xarm.py`](../run_dwa_then_xarm.py) | 주행 프로그램이 성공으로 종료된 뒤 [YOLO·xArm 런치](../local_ws/src/use_yolo/launch/vision_pick_drop.launch)를 실행 |

## RGB-D 인식과 파지

팔이 초기 관측 자세에 도착하면 `/vision_enable`을 켭니다. [검출 노드](../local_ws/src/use_yolo/scripts/yolo_detector.py)는 L515의 RGB 영상에서 OpenCV DNN으로 YOLOv8n ONNX를 실행합니다. 검출 클래스는 `red_box`, `yellow_box`, `blue_box`이며, 박스 안의 깊이 영상에서 상자 상대 위치를 계산합니다. 팔이 움직이는 동안 새 목표 TF 발행을 멈춥니다.

[파지 노드](../local_ws/src/use_yolo/scripts/xarm_vision_pick_drop.py)는 초기 관측 자세에서 카메라 상대 좌표, 팔의 현재 위치와 장착 오프셋·스케일을 조합합니다. 관측 자세를 바꾸면 좌표 대응을 다시 확인해야 합니다. 수직 파지 IK는 목표의 수평 위치로 J1을 결정하고, J5 연동값을 정한 뒤 수치 자코비안과 감쇠 최소제곱으로 J2~J4를 계산합니다. 팔은 접근, 하강, 파지, 상승, 적재, 관측 자세 복귀 순으로 동작합니다.

주행에서 파지로 전환되지 않았던 문제와 RGB 입력 끊김의 확인 과정은 [통합 시험 문제 해결](INTEGRATION_DEBUGGING.md)에 정리했습니다. 반복 횟수, 성공률, 위치 오차, 처리 시간의 별도 정량 자료는 없습니다.
