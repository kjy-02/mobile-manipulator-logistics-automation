# 외부 의존성과 재현 환경

## 현재 PC

준비 스크립트는 기본적으로 `~/catkin_ws/devel/setup.bash`를 적용한 다음 이 저장소의 `local_ws`를 빌드합니다. 따라서 기존에 동작하던 드라이버·위치 추정 환경을 재사용합니다.

```bash
SCOUT_BASE_SETUP=/다른/기반_workspace/devel/setup.bash bash start_all.sh
```

실행용 세 패키지는 반드시 이 저장소의 `local_ws`에서 선택돼야 하며, 실행 스크립트가 경로를 확인합니다. 기반 workspace의 다른 동일 이름 패키지를 사용하지 않습니다.

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

실행 중인 PC에서 확인한 저장소 주소와 커밋은 루트의 `dependencies.repos`에 있습니다. 패키지 경로·로컬 변경 상태·xArm SDK 서브모듈 커밋은 `dependency_snapshot.json`, 설치된 ROS·관련 시스템 패키지 버전은 `installed_packages.tsv`에 기록했습니다.

## 새 PC에서 구성할 때

ROS Noetic와 빌드 도구·`vcstool`·`rosdep`이 설치된 환경에서, 별도 기반 workspace를 만들고 아래처럼 외부 소스를 가져올 수 있습니다. 이 예시는 설치 절차의 출발점이며 새 PC에서 전체 설치 성공을 검증한 자동 설치기는 아닙니다.

```bash
mkdir -p ~/robot_dependencies/src
vcs import ~/robot_dependencies/src < ~/scout_dwa_xarm/dependencies.repos
```

각 저장소의 서브모듈도 초기화해야 합니다. `xarm_ros` SDK 등은 서브모듈을 사용합니다. 설치된 Noetic 패키지와 중복 소스 패키지가 생기는 경우 기반 workspace 구성을 확인해야 합니다.

```bash
source /opt/ros/noetic/setup.bash
rosdep install --from-paths ~/robot_dependencies/src ~/scout_dwa_xarm/local_ws/src --ignore-src -r -y
```

`rosdep`만으로 모든 의존성이 해결된다고 가정하지 마세요. librealsense, 하드웨어 드라이버 및 upstream 패키지의 추가 요구사항은 각 프로젝트 문서를 확인합니다.

기반 workspace를 빌드한 뒤 배포본의 `start_all.sh`에 `SCOUT_BASE_SETUP`을 지정합니다. `start_all.sh`는 준비 단계에서도 실제 하드웨어 드라이버와 팔 제어기를 시작하므로, 빌드만 하려면 다음 명령을 사용합니다.

```bash
source ~/robot_dependencies/devel/setup.bash
catkin_make -C ~/scout_dwa_xarm/local_ws -j2 -DCATKIN_ENABLE_TESTING=OFF -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## 현재 기반 workspace의 로컬 변경

외부 저장소의 커밋만 가져오면 현재 PC의 미커밋 수정까지 복원되지는 않습니다. 확인된 tracked 변경은 `dependency_patches/`에 기록했습니다. PCD 지도 차이는 패치에서 제외하고 이 배포본의 `maps`에 실제 파일을 담았습니다. 미추적 파일·모든 시스템 설정을 포함한 전체 시스템 백업은 아닙니다.

패치는 해당 저장소의 기록된 커밋에 대해 내용을 검토하고 `git apply --check`로 확인한 후 적용합니다. xArm SDK 서브모듈 정보는 JSON에 별도로 남겼습니다. 이번 묶음은 현재 PC의 실제 기반 workspace를 대체하는 검증된 전체 드라이버 배포본은 아닙니다.
