# 멀티3층 목표 주행 실행

지도: `/home/rilab/catkin_ws/map/my_map.pcd` (멀티3층).
보존한 원본 AMSL DWA commit: `6387a9660c2064fa0545ac81d55f4bbe7962651f`.
현재 목표 주행은 보존한 원본 `dwa_planner` 실행 파일을 사용한다.
mission은 `low_speed_planner.launch`를 시작하며 개선판은 현재 실행하지 않는다.
단일 목표까지 연속 주행하며, 도착 시 정지하고 도착 로그를 출력한 뒤 주행 프로그램과 해당 DWA를 종료한다.
이 구성의 전체 목표 도착 실차 시험은 아직 수행하지 않았다.

## 현재 켜진 환경에서 실행

ROS master, Scout 드라이버, LiDAR, 센서 정적 TF는 기존 실행을 사용한다.
HDL은 멀티3층 지도로 재시작했다. 새 지도에서 초기 위치를 다시 지정한다.
아래 launch는 이전 `dwa_map.launch`, `low_speed_field_test.launch`, `first_motion_checks.launch`와 중복 실행하지 않는다.
기존 DWA와 first_motion_checks 실행은 이번 지도 변경 과정에서 종료했다.

터미널 1 — 장애물 변환, 명령 감시, footprint, 독립 명령 watchdog:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch scout_dwa goal_navigation.launch accepted_hardware_profile:=true accepted_obstacle_input:=true
```

RViz에서 Fixed Frame=`map`, PointCloud2 토픽=`/globalmap`을 선택하고,
`2D Pose Estimate`로 멀티3층 지도에서 실제 현재 위치와 방향을 지정한다.
지도와 실시간 LiDAR가 맞는지 사용자가 확인한다.
필요하면 RViz만 다음 명령으로 실행한다.

```bash
source ~/catkin_ws/devel/setup.bash
rviz -d ~/catkin_ws/src/scout_dwa/rviz/scout_dwa.rviz
```

터미널 2 — 목표 대기 및 실제 차량 주행, 로그 저장:

```bash
source ~/catkin_ws/devel/setup.bash
set -o pipefail
rosrun scout_dwa run_goal_navigation.py --run --max-seconds 1800 --goal-wait-seconds 1800 2>&1 | tee /tmp/scout_goal_navigation.log
```

위 프로그램의 목표 대기 로그가 나오면 RViz `2D Nav Goal`로 목표 위치와 최종 방향을 지정한다.
목표 수신 후 실제 차량 출력이 연결된다. 최대 주행 시간은 30분이며, 그 전에 도착하면 바로 종료한다.
다음 목표는 터미널 2 명령을 다시 실행하고 새 목표를 지정한다.
주행 중 목표나 초기 위치를 변경하면 정지하고 종료한다. 수동 중단은 터미널 2에서 `Ctrl+C`.

도착 로그:

```text
목표 지점 도착! 정지 후 주행 프로그램을 종료합니다.
```

주행 프로그램이 종료되어도 터미널 1의 watchdog은 0 속도를 유지한다.
로그는 `/tmp/scout_goal_navigation.log`, 원본 planner 로그는 프로그램이 출력하는
`/tmp/scout_goal_planner_*.log` 경로에 저장된다.

## 모두 꺼진 상태에서 시작

각 명령은 별도 터미널에서 실행하며, 모든 터미널에서 먼저
`source ~/catkin_ws/devel/setup.bash`를 실행한다. 이미 실행 중인 항목은 중복 실행하지 않는다.

```bash
# 터미널 0
roscore

# 터미널 1: CAN 설정 후 Scout 드라이버
sudo ip link set can0 up type can bitrate 500000
roslaunch scout_dwa scout_base_for_dwa.launch is_scout_mini:=false is_scout_omni:=false pub_tf:=false

# 터미널 2: LiDAR
roslaunch scout_dwa lidar.launch

# 터미널 3: 멀티3층 지도
roslaunch scout_dwa hdl_localization_map.launch

# 터미널 4: 기존 센서 변환을 정적 TF로 발행
roslaunch scout_dwa legacy_sensor_tf.launch

# 터미널 5: 주행 연결
roslaunch scout_dwa goal_navigation.launch accepted_hardware_profile:=true accepted_obstacle_input:=true

# 터미널 6: RViz
roslaunch scout_dwa visualization.launch
```

이후 초기 위치 지정 → 위 목표 주행 프로그램 실행 → 새 목표 지정 순서로 진행한다.

## 적용된 동작과 남은 조건

- DWA 입력은 각도 제한 없는 `/scout_dwa/local_map`이다.
- 현재 목표 주행의 0.61m 외접원 정지와 별도 회전 원형 정지는 제거됐다. guard는 전체 `/scout_dwa/local_map`으로 실제 직사각형 footprint의 진행·회전 영역을 검사한다. ±60° front_obstacles 토픽은 비교용으로 계속 발행하지만 현재 목표 주행 guard 입력이 아니다.
- DWA 내부 점수 정규화·목표 비용·장애물 비용·지도 추출·후보 생성・충돌 검사는 원본 방식이다. 개선판의 연속 차체 검사와 최종 회전 내부 검사는 사용하지 않는다.
- 목표 선속도는 0.55 m/s, 주행 yaw 명령 상한은 1.0 rad/s다. 제자리 회전 속도는 최소 0.45, 최대 0.9 rad/s로 원본의 1.5배다. 나머지 속도·가속도 설정은 원본 GitHub 값과 동일하다.
- 개선판의 별도 조향 비용은 원본 실행에 적용되지 않는다.
- 도착 판정은 원본 finish 신호와 실제 map 위치·방향 오차(0.5 m, 0.1 rad)를 함께 확인한다.
- 장애물로 정지하면 입력이 정상인 동안 제한 시간 내에서 재개를 기다린다. 입력 장애가 지속되면 정지·종료한다.
- 두 accepted 인자는 사용자가 수락한 기존 장착·footprint·입력을 선택한다. 미측정 모델·센서 보정·전체 감지 영역·차량 firmware watchdog을 검증했다고 의미하지 않는다.
- 새 지도에서의 위치 정합, 목표까지 도착, 실제 정지 성능은 사용자 현장 확인이 남아 있다.
- guard의 경로 검사 시간은 반응 시간 0.3초 + 측정 선속도/가정 감속도 1m/s²다. 감속도와 반응 시간은 소프트웨어 가정이며 실측 제동거리 보장이 아니다.
- 이번 원본 실행 복원 후 실제 주행은 실행하지 않았다. 이전 사용자 시험은 당시 사용한 실행판과 설정을 기준으로 해석해야 한다.
