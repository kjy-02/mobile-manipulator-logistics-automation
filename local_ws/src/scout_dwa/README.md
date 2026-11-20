# scout_dwa: AMSL DWA environment integration

2026-10-03 설정에서는 `run_goal_navigation.py`가 원본
`dwa_planner`를 `low_speed_planner.launch`로 실행하도록 복원했다.
`src/improved_dwa`는 비교용으로 남아 있으며 현재 목표 주행에는 사용하지 않는다.
외부 출력 guard의 차체 경로 검사는 유지한다. 원본 저장소는 수정하지 않았다.

이 패키지는 AMSL의 `dwa_planner` 원본 알고리즘을 Scout ROS 환경에 연결한다.
원본은 별도 패키지 `/home/rilab/catkin_ws/src/dwa_planner`에 고정되어 있고,
센서 변환, footprint 발행, 토픽 remap, 명령 감시는 이 패키지에만 있다.

현재 상태와 미확인 항목은 [IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md),
설계 기준은 [AMSL_DWA_MIGRATION_PLAN.md](AMSL_DWA_MIGRATION_PLAN.md)를 따른다.

**현재 멀티3층 지도에서 목표까지 주행하는 실행 명령은
[RUN_GOAL_NAVIGATION.md](RUN_GOAL_NAVIGATION.md)를 사용한다.**
아래에는 당시 연결 및 시험 구성을 기록했다. 현재 통합 프로젝트의 실행 순서는 [저장소 루트 README](../../../README.md)를 참고한다.

당시 기본 연결에는 `scout_dwa.launch`를 사용했다. 이 launch는 GitHub
원본의 `dwa_planner/launch/local_planner.launch`를 직접 include하고,
LiDAR 변환·footprint·토픽 remap·출력 guard만 연결한다. 기존 확장 세션 구성은
`amsl_dwa_integration.launch`에 남아 있으며 기본 실물 연결에 요구하지 않는다.
`roslaunch scout_dwa scout_dwa.launch`는 기본적으로 출력이 차단된다.

Noetic 비교본 반영 이후의 구현 순서와 결정은
[AMSL_DWA_NEXT_IMPLEMENTATION_PLAN.md](AMSL_DWA_NEXT_IMPLEMENTATION_PLAN.md)를 따른다.

## 안전한 소프트웨어 검증

```bash
cd /home/rilab/catkin_ws/src/scout_dwa
./scripts/verify_upstream.sh

cd /home/rilab/catkin_ws
catkin_make --pkg dwa_planner scout_dwa -DCMAKE_BUILD_TYPE=Release -j2
catkin_make run_tests_scout_dwa
catkin_test_results build/test_results/scout_dwa
```

같은 검증은 `./scripts/run_synthetic_tests.sh`로 재실행할 수 있다. 설정 파일은
실행 전에 `validate_integration_config.py`로 검사한다.
rostest는 각 launch에 전용 master를 생성하고 종료한다. 재현 스크립트는 ROS_HOME을
/tmp 아래에 두며 전체 실행에 300초 제한을 적용한다. 빌드 target의 성공만으로
시험 성공을 판단하지 않고 마지막 `catkin_test_results`의 오류·실패 수도 확인한다.

합성 map 입력을 직접 관찰하려면 다음 launch를 실행한다. 모든 명령 출력은
`/scout_dwa/cmd_vel_safe`까지이며 실제 Scout의 `/cmd_vel`에는 연결되지 않는다.

```bash
source /opt/ros/noetic/setup.bash
source /home/rilab/catkin_ws/devel/setup.bash
roslaunch scout_dwa synthetic_pipeline.launch obstacle_mode:=map
```

`obstacle_mode:=scan`으로 동일한 합성 입력의 LaserScan 경로를 시험할 수 있다.

## 실제 환경 연결 전 구성

### 기존 실행 명령에서 반영한 연결

당시 사용한 지도 이름과 경로:

- **멀티3층 맵 (현재 기본)**: `~/catkin_ws/map/my_map.pcd`
- **하드실 맵**: `~/catkin_ws/src/hdl_localization/data/map/hard.pcd`
- **야외 학생회관 맵 (기존)**: `~/catkin_ws/src/hdl_graph_slam/map.pcd`

다른 지도는 `hdl_localization_map.launch map_pcd_path:=절대경로`로 선택한다.

현재 ROS master와 VLP16 수신 노드는 이미 실행 중이다. 중복 실행하지 않는다.
CAN도 UP, 500000 bitrate와 차량 상태 프레임 수신이 확인됐다.

당시 Scout 2.0을 장비 모델 후보로 기록했으며, 실제 모델은 확인 전이었다. 모델을 확인한 뒤
기존 `scout_robot_base.launch` 대신 명시적으로 모델·odom 토픽을 전달하는 구성을 사용한다.
기존 bringup의 Mini 인자는 하위 launch에 전달되지 않아 실제 기본값은 false다.

기존 센서 좌표값을 유지하는 연결은 `map -> velodyne -> base_link`다.
1 Hz 동적 TF의 센서 시각 조회 지연을 피하기 위해 스크립트 대신
`roslaunch scout_dwa legacy_sensor_tf.launch`를 실행한다. 둘을 함께 실행하지 않는다.
이 경우 Scout는 wheel odom 토픽만 발행하고 TF는 끈다. 드라이버 시작은 SDK의
`EnableCommandedMode()`를 호출하므로 현재 관측 작업에서는 시작하지 않았다.
아래 명령은 모델 확인 후 별도 드라이버 운용 단계에서 사용한다.

```bash
roslaunch scout_dwa scout_base_for_dwa.launch is_scout_mini:=false is_scout_omni:=false pub_tf:=false
```

위 구성은 wheel odom을 `/scout/odom`으로 발행한다. HDL은 기존 명령과 동일하게
`roslaunch scout_dwa hdl_localization_map.launch`로 실행하며 pose 토픽은
`/localization/odom`으로 분리된다. 기존 TF 스크립트의 -0.35 m는 기존 설정값이며
실측 확인값으로 간주하지 않는다. `map -> base_link`가 연결됐는지 확인한다.

RViz는 기존 설정을 사용하고 Fixed Frame을 `map`으로 둔다. `2D Pose Estimate`로
초기 위치를 지정하고 `2D Nav Goal`로 단일 목표를 전달한다. 새 DWA 입력 확인 명령은:

```bash
roslaunch scout_dwa dwa_map.launch
```

이는 원본 AMSL launch를 직접 포함하며 실제 `/cmd_vel`에는 출력하지 않는다.
기존 `waypoints:=...` 인자는 새 launch에서 지원하지 않는다. 연속 목표 실행은
별도 waypoint 전달 기능이 필요하다. 현재 기본 출력 guard의 localization health도
별도 monitor가 필요하므로 TF 연결만으로 실주행이 활성화되지 않는다.

다음 값은 실물 또는 신뢰할 수 있는 기구 설정으로 먼저 확정해야 한다.

- Scout V2, Mini, Omni 중 실제 모델
- `base_link -> velodyne`의 x, y, z, roll, pitch, yaw
- 바퀴와 탑재물을 포함한 footprint
- 라이다 높이 필터, 차체 self filter, 유효 감지 범위
- 실제 허용 속도, yaw rate, 가속도와 감속도
- `/scout/odom`, `/velodyne_points`, TF의 실제 갱신 주기

[config/dwa_scout_reference.yaml](config/dwa_scout_reference.yaml),
[config/cloud_adapter_reference.yaml](config/cloud_adapter_reference.yaml),
[config/footprint_scout_v2_reference.yaml](config/footprint_scout_v2_reference.yaml)은
합성 및 정적 연결 검토용 참조값이다. 검증된 실차 보정값으로 취급하지 않는다.

## 통합 launch의 기본 동작

```bash
roslaunch scout_dwa amsl_dwa_integration.launch
```

기본값은 다음과 같다.

- AMSL 원본 출력: `/scout_dwa/cmd_vel_raw`
- guard 출력: `/scout_dwa/cmd_vel_safe`
- `enable_motion_output:=false`: guard가 0만 발행
- hardware_verified=false, coverage_confirmed=false: enable 서비스 호출도 실측 gate를 해제하지 않음
- 실제 Scout `/cmd_vel`로 remap하지 않음
- DWA odometry: `/scout/odom`
- HDL localization odometry: `/localization/odom`
- 장애물 입력 기본 모드: robot centered OccupancyGrid

실차 운용 launch와 `/cmd_vel` 연결은 물리값 확인, 현장 preflight, 제한 속도
검증 및 별도 운용 지시가 함께 있을 때 구성한다.

## 노드 역할

DWA 계산은 각도 제한 없이 `/scout_dwa/local_map`을 사용한다.
근접 출력 정지는 `/scout_dwa/front_obstacles`의 정면 **±60도** 점을 사용한다.
adapter의 `planner_front_only`와 `planner_half_angle_deg`는 각도 필터 설정이며,
현재 필터 출력의 용도는 DWA 입력이 아니라 근접 정지 지도다.
`map_topic`은 front_obstacles, `safety_map_topic`은 local_map으로 연결한다.

- `cloud_obstacle_adapter`: PointCloud2를 메시지 시각의 TF로 `base_link`에
  변환하고 OccupancyGrid 또는 LaserScan을 생성한다.
- `static_footprint_publisher`: 유효한 base_link 다각형을 latched 발행한다.
- `cmd_vel_guard`: raw DWA 명령을 그대로 통과시키거나 0으로 바꾼다. 입력
  freshness, TF freshness, input/sensor/localization health, 유한값과 속도 범위,
  enable 상태를 검사하고 차단 이유를 `~reason`에 발행한다.
- `dwa_planner`: 고정된 upstream executable을 그대로 사용한다.
- `planner_preflight.py`: 유효한 최종 설정을 확인한 뒤 원본 executable을 시작한다.
- `input_gateway.py`: 채널별 오류를 유지하고 속도 하향 갱신은 fresh stopped odom 이후 전달한다.
- `localization_session.py`: 초기 pose·재위치추정 전에 guard disable과 localization inhibit를 적용한다.

## 주행 없는 실제 입력 확인

이미 실행 중인 ROS master에서만 다음 읽기 전용 점검을 사용할 수 있다.
하드웨어 노드나 CAN을 기동하지 않고, 제어 명령을 발행하지 않는다.

```bash
source /home/rilab/catkin_ws/devel/setup.bash
rosrun scout_dwa read_only_preflight.py --duration 5
```

현재 관측 결과는 `validation/2026-09-30/live_preflight.json`에 있다.
ROS master가 없는 경우 exit 2와 연결 오류를 기록한다. 관측된 TF·속도·품질 값도
실측 승인값으로 자동 승격하지 않는다.

## 목표 교체와 localization 세션

새 목표가 기존 목표와 다르면 gateway는 전달을 보류하고 input health를 false로 둔다.
정지 odometry가 확인된 후 `/scout_dwa/planner_preflight/restart_session` Trigger 서비스를
사용한다. wrapper는 guard disable → 기존 원본 종료 → 새 목표 latch → 새 원본 시작 순서로
진행한다. 성공 후에도 guard는 disabled다.

초기 pose는 `/localization_session/initial_pose`로 전달하고 재위치추정은
`/localization_session/relocalize` Trigger 서비스로 요청한다. 품질 재검사는
`/localization_session/allow_quality_recheck`로 명시적으로 시작한다.
세션은 주행을 자동 재개하지 않는다. HDL의 직접 `/initialpose`·`/relocalize` 호출을
모든 경우 감지할 수 있다고 가정하지 않으며 wrapper 우회 운용은 지원하지 않는다.

`localization_monitor.launch`는 품질과 map→odom 보정 jump 임계값을 필수 인자로 요구한다.
보정값 jump가 발생하면 health는 false로 유지되고 명시적 세션 재검사가 필요하다.
odom→base의 정상 바퀴 이동은 map→odom 보정 검사와 분리된다.

## 확인된 원본 한계

`validation/2026-09-30/sampling_limits.json`은 float32 산술로 원본 샘플링을 재현한
진단 자료다. 단독 scan bin 또는 map cell이 누락될 수 있다. `thin_scan_limit.test`는
실제 원본 실행으로 scan bin 1 누락을 재현하고 현재 footprint guard의 차단을 확인한다.
앞으로 이동할 경로의 얇은 장애물 누락을 해결했다고 해석하지 않는다.
목표 회전 중 목표 교체와 반복 finish_flag도 합성시험에서 원본 제약으로 기록한다.

기존 [src/dwa_navigator.cpp](src/dwa_navigator.cpp)는 백업으로 남아 있지만
CMake target과 기본 launch에서 제외되어 실행 바이너리가 생성되지 않는다.
