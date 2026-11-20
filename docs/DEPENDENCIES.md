# 설치와 실행 환경

실내 통합 시연은 Ubuntu 20.04의 ROS 1 Noetic에서 실행했습니다. 기존 로봇의 Scout, Velodyne, HDL Localization, RealSense, xArm, MoveIt 환경이 준비돼 있다면 그 기반 워크스페이스를 그대로 사용할 수 있습니다.

| 장비·기능 | 필요한 환경 |
| --- | --- |
| SCOUT 2.0 | `scout_base`, `scout_msgs`, `ugv_sdk`, SocketCAN |
| Velodyne VLP-16 | `velodyne_pointcloud`, `velodyne_driver` |
| 위치 추정·지도 작성 | `hdl_localization`, `hdl_global_localization`, `ndt_omp`, `fast_gicp`; 지도 작성 시 `hdl_graph_slam`과 g2o |
| RealSense L515 | `realsense2_camera`, librealsense, `cv_bridge`, OpenCV DNN |
| xArm5 | `xarm_ros`과 SDK 서브모듈, MoveIt, ROS Control |

## 기존 기반 워크스페이스 사용

`start_all.sh`는 기본적으로 `~/catkin_ws/devel/setup.bash`를 사용해 `local_ws`의 세 패키지를 빌드합니다. 경로가 다르면 저장소 루트에서 다음처럼 지정합니다.

```bash
SCOUT_BASE_SETUP=/path/to/base_ws/devel/setup.bash bash start_all.sh --map-pcd "$PWD/maps/m304.pcd"
```

## 기반 워크스페이스 준비

ROS Noetic, `catkin_make`, `vcstool`, `rosdep`을 준비한 뒤, 저장소 루트에서 아래 명령으로 [고정 버전](../dependencies.repos)의 외부 소스를 가져옵니다. 설치 환경에 따라 [RealSense ROS](https://github.com/IntelRealSense/realsense-ros)와 MoveIt을 별도로 설치하고, 각 외부 패키지의 추가 의존성을 확인합니다. 사용 환경의 RealSense ROS 패키지 버전은 2.3.2였습니다.

```bash
source /opt/ros/noetic/setup.bash
base_ws="$HOME/robot_dependencies"
mkdir -p "$base_ws/src"
vcs import "$base_ws/src" < dependencies.repos
git -C "$base_ws/src/xarm_ros" submodule update --init --recursive
rosdep install --from-paths "$base_ws/src" local_ws/src --ignore-src -r -y
catkin_make -C "$base_ws"
```

실차 환경의 xArm SDK C++ 서브모듈은 `fa91a43794c12b84821058635ad65b955775892f`를 사용했습니다. 새 PC의 전체 의존성 설치와 실차 구동은 위 명령 순서로 따로 검증하지 않았습니다.

지도 작성은 [별도 SLAM 런치](../launch/mapping.launch)를 사용했습니다. 당시 VLP-16의 nodelet manager가 실행된 상태에서 사용한 설정으로, 저장 지도를 읽는 통합 주행과 별개입니다.

빌드만 하려면 기반 환경을 적용한 상태에서 `catkin_make -C local_ws -DCATKIN_ENABLE_TESTING=OFF`를 실행합니다. 통합 운용 순서는 [README](../README.md)를 참고하세요.
