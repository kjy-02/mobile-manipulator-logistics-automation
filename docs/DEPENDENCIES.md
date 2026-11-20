# 외부 의존성과 재현 환경

## 기존 ROS 워크스페이스에서 실행

준비 스크립트는 기본적으로 `~/catkin_ws/devel/setup.bash`를 적용한 다음 이 저장소의 `local_ws`를 빌드합니다. 따라서 기존에 동작하던 드라이버·위치 추정 환경을 재사용합니다.

```bash
SCOUT_BASE_SETUP=/다른/기반_workspace/devel/setup.bash bash start_all.sh
```

실행 스크립트는 `scout_dwa`, `dwa_planner`, `use_yolo`가 이 저장소의 `local_ws`에서 선택됐는지 확인합니다. 기반 워크스페이스에 같은 이름의 패키지가 있어도 프로젝트 쪽 패키지를 사용합니다.

## 필요한 구성

| 역할 | 주요 패키지·소프트웨어 |
|---|---|
| 차량 | `scout_base`, `scout_msgs`, `ugv_sdk`, SocketCAN |
| LiDAR | `velodyne_driver`, `velodyne_pointcloud` 및 관련 패키지 |
| 위치 추정 | `hdl_localization`, `hdl_global_localization`, `ndt_omp`, `fast_gicp` |
| 팔 | `xarm_ros`의 `xarm5_moveit_config`, `xarm_bringup`, `xarm_api`, `xarm_msgs`, SDK 등 |
| 제어 | MoveIt, ROS control, ros_controllers 및 메시지 패키지 |
| 카메라 | `realsense2_camera`, librealsense |
| Python | OpenCV DNN, NumPy, cv_bridge, rospy, rospkg, PyYAML 등 |

외부 저장소 주소와 사용한 커밋은 루트의 `dependencies.repos`에 있습니다. 패키지 경로, 로컬 변경, xArm SDK 서브모듈 커밋은 `dependency_snapshot.json`에 기록했습니다. `installed_packages.tsv`에는 당시 PC에 설치된 ROS와 관련 시스템 패키지의 버전이 있습니다.

## 새 PC에서 구성할 때

ROS Noetic, 빌드 도구, `vcstool`, `rosdep`을 설치한 뒤 별도 기반 워크스페이스를 구성합니다. 아래 명령은 외부 소스를 가져오는 예시입니다. 새 PC에서의 전체 설치 과정은 아직 검증하지 않았습니다.

```bash
mkdir -p ~/robot_dependencies/src
vcs import ~/robot_dependencies/src < ~/scout_dwa_xarm/dependencies.repos
```

각 저장소의 서브모듈도 초기화해야 합니다. `xarm_ros` SDK 등은 서브모듈을 사용합니다. 설치된 Noetic 패키지와 중복 소스 패키지가 생기는 경우 기반 workspace 구성을 확인해야 합니다.

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths ~/robot_dependencies/src ~/scout_dwa_xarm/local_ws/src --ignore-src -r -y
```

librealsense와 하드웨어 드라이버에는 `rosdep` 외에 설치나 설정이 필요한 항목이 있을 수 있습니다. 각 패키지의 설치 문서도 확인합니다.

기반 workspace를 빌드한 뒤 배포본의 `start_all.sh`에 `SCOUT_BASE_SETUP`을 지정합니다. `start_all.sh`는 준비 단계에서도 실제 하드웨어 드라이버와 팔 제어기를 시작하므로, 빌드만 하려면 다음 명령을 사용합니다.

```bash
source ~/robot_dependencies/devel/setup.bash
catkin_make -C ~/scout_dwa_xarm/local_ws -j2 -DCATKIN_ENABLE_TESTING=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## 기반 워크스페이스에서 사용한 로컬 수정

기록된 외부 저장소 커밋 이후 로컬에서 수정한 추적 파일은 `dependency_patches/`에 있습니다. PCD 지도는 패치에서 빼고 `maps/`에 원본 파일을 담았습니다. 미추적 파일과 PC 전체 설정은 이 기록에 포함되지 않습니다.

각 패치는 기록된 커밋에서 `git apply --check`로 확인한 뒤 적용합니다. xArm SDK 서브모듈 정보는 `dependency_snapshot.json`에 있습니다. 패치와 버전 목록은 당시 구성 기록이며, 실제 구동에 쓸 전체 드라이버 구성은 기반 워크스페이스에서 별도로 준비합니다.
