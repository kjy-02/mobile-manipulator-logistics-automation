# Scout ROS1 Noetic: HDL localization + AMSL DWA 실행 가이드

이 폴더는 `amslabtech/dwa_planner` 원본 commit
`6387a9660c2064fa0545ac81d55f4bbe7962651f`을 Scout/VLP-16 실차 구성에
맞춘 전송용 사본이다.

## 적용한 실차 정보

| 항목 | 적용값 |
|---|---|
| ROS | ROS1 Noetic |
| 글로벌 frame | `map` |
| 로봇 frame | `base_link` |
| LiDAR frame 후보 | `velodyne` |
| PointCloud2 | `/velodyne_points` |
| RViz goal | `/move_base_simple/goal` |
| Scout 속도 odometry | `/scout_odom` |
| Scout 명령 | `/cmd_vel` |
| 기존 선속도 범위 | `0.30 .. 0.50 m/s` |
| 적용 시작 목표속도 | `0.30 m/s` |
| 적용 최대속도 | `0.50 m/s` |
| 적용 최대 yawrate | 대칭 `0.70 rad/s` |
| 목표 도착 거리 | `0.50 m` |
| 기존 장애물 정지 거리 | `0.45 m` |
| 임시 원형 모델 | 반경 `0.45 m` + padding `0.05 m` |
| cloud 필터 | `x > -0.45`, `-0.55 < z < 0.50` |

기존 코드의 `abs(y) > 0.17` 필터는 정면 장애물을 삭제할 수 있어 반영하지
않았다. PointCloud2는 TF를 통해 `base_link`로 변환한 후 약 5도 간격으로 가장
가까운 점만 DWA에 전달한다. 따라서 `base_link <-> velodyne` 오프셋도 TF를
통해 반영된다.

## 1. 실차로 복사 및 빌드

이 폴더의 `dwa_planner` 디렉터리를 실차의 catkin workspace로 복사한다.

```bash
cp -r dwa_planner ~/catkin_ws/src/dwa_planner
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source ~/catkin_ws/devel/setup.bash
```

동일한 이름의 기존 `dwa_planner`가 있다면 덮어쓰기 전에 별도로 백업하고,
두 복사본을 동시에 `src` 아래에 두지 않는다.

## 2. 변경된 전체 실행 순서

### 터미널 0: ROS master

```bash
source ~/catkin_ws/devel/setup.bash
roscore
```

### 터미널 1: CAN

```bash
source ~/catkin_ws/devel/setup.bash
rosrun scout_bringup bringup_can2usb.bash
```

### 터미널 2: Scout base

Scout odometry를 HDL odometry와 분리하기 위해 이름을 `/scout_odom`으로 바꾼다.

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch scout_bringup scout_robot_base.launch odom_topic_name:=/scout_odom
```

실차에 설치된 launch가 `odom_topic_name` 인자를 노출하지 않으면 해당 launch의
`scout_base_node`에 같은 이름의 param을 추가해야 한다. 임의로 진행하지 말고
먼저 다음 결과를 확인한다.

```bash
roscd scout_bringup
grep -R "odom_topic_name" launch
rostopic info /scout_odom
rostopic echo -n 1 /scout_odom/twist
```

`/scout_odom.twist.twist.linear.x`와 `angular.z`가 실제 차량 움직임을 반영해야 한다.

### 터미널 3: VLP-16

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch velodyne_pointcloud VLP16_points.launch
```

확인:

```bash
rostopic hz /velodyne_points
rostopic echo -n 1 /velodyne_points/header
```

### 터미널 4: 기존 HDL localization

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch scout_dwa hdl_localization_map.launch
```

HDL localization이 `/odom` pose를 발행해도 DWA는 이를 사용하지 않는다. DWA는
속도가 포함된 `/scout_odom`을 사용한다.

### 터미널 5: 기존 Velodyne TF

```bash
source ~/catkin_ws/devel/setup.bash
rosrun tf_code velodyne_to_baselink.py
```

확인:

```bash
rosrun tf tf_echo map base_link
rosrun tf tf_echo base_link velodyne
```

두 TF가 모두 갱신되어야 한다. `map -> base_link`가 없으면 DWA를 실행하지 않는다.

### 터미널 6: RViz

```bash
source ~/catkin_ws/devel/setup.bash
roscd hdl_localization/rviz
rviz -d hdl_localization.rviz
```

RViz에서:

1. `Fixed Frame = map`
2. PointCloud2 `/globalmap` 추가
3. PointCloud2 `/velodyne_points` 추가
4. `2D Pose Estimate`로 현재 위치와 방향 지정
5. localization 정합이 안정된 뒤에만 다음 단계 진행

### 터미널 7: DWA

첫 시험은 Scout에 연결하지 않고 테스트 토픽으로 출력한다.

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch dwa_planner scout_velodyne.launch cmd_vel:=/cmd_vel_test
```

RViz의 `2D Nav Goal`로 가까운 목표를 지정하고 다음을 확인한다.

```bash
rostopic echo /cmd_vel_test
rostopic hz /scout_dwa/candidate_trajectories
rostopic hz /scout_dwa/selected_trajectory
```

후보 궤적과 속도 방향이 맞으면 노드를 종료하고 실차 출력으로 다시 실행한다.

```bash
roslaunch dwa_planner scout_velodyne.launch cmd_vel:=/cmd_vel
```

## 3. 실차 출력 전 필수 확인

```bash
rostopic info /cmd_vel
rostopic info /scout_odom
rostopic info /odom
rostopic hz /scout_odom
rostopic hz /velodyne_points
rosrun tf tf_echo map base_link
rosrun tf tf_echo base_link velodyne
```

통과 조건:

- `/scout_odom` publisher는 Scout base이고 twist가 0으로 고정되지 않는다.
- `/odom`과 `/scout_odom`이 분리되어 있다.
- `/velodyne_points.header.frame_id`에서 `base_link`로 TF 변환할 수 있다.
- `map -> base_link`가 안정적으로 갱신된다.
- `/cmd_vel`에 다른 자율주행/teleop publisher가 동시에 붙어 있지 않다.
- RC와 E-stop으로 즉시 정지할 수 있다.

## 4. 파라미터 파일

- `dwa_planner/config/scout_dwa_param.yaml`
- `dwa_planner/config/scout_robot_param.yaml`

최초 실차 출력은 `TARGET_VELOCITY=0.30 m/s`로 제한했다. 기존 코드의 최고속도
`0.50 m/s`는 `MAX_VELOCITY`로만 보존했다. 저속 시험이 통과하기 전에 목표속도를
올리지 않는다.

현재 `ROBOT_RADIUS=0.45 m`는 기존 장애물 임계값에서 가져온 임시 원형 모델이다.
실제 Scout, 범퍼, 센서 및 적재물을 측정한 polygon footprint로 교체해야 최종
충돌 안전성을 주장할 수 있다.

## 5. 중단 조건

다음 중 하나라도 발생하면 `/cmd_vel` 0을 발행하고 원인을 해결한다.

- PointCloud TF unavailable 경고
- `/scout_odom` 갱신 중단
- `/velodyne_points` 갱신 중단
- `map -> base_link` 단절
- HDL localization 미수렴 또는 pose 점프
- RViz에서 장애물이 로봇 반대 방향에 표시됨
- 정지 상태인데 `/scout_odom` twist가 크게 흔들림
- 다른 노드가 동시에 `/cmd_vel` 발행

이 DWA는 센서가 몇 제어주기 동안 갱신되지 않으면 0 속도를 발행하지만, Scout
실차에는 별도의 command timeout/safety mux를 추가하는 것이 권장된다.
