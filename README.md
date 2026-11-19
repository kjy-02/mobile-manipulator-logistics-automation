# 모바일 매니퓰레이터 물류 자동화 시스템

SCOUT 2.0과 xArm5를 결합해 **실내 작업대의 상자를 분류하고 차량 측면 트레이에 순서대로 적재**하는 3인 팀 졸업작품입니다.

**[실내 통합 시연 바로 보기](https://app.notion.com/p/41c18a17dfa6816c8ad6ec392436eaca)**

김재윤은 3D LiDAR 지도 작성, HDL Localization 설정, DWA 주행 기능 수정, 전체 통합 시험을 담당했습니다.

## 작업 흐름

1. RViz에서 초기 자세와 픽업존 목표를 지정합니다. 저장된 실내 지도 `maps/m304.pcd`에서 위치를 추정하고 DWA로 픽업존에 접근합니다.
2. DWA의 주행 완료 신호를 받으면 차량을 정지시키고 주행 프로그램을 종료한 뒤 상자 인식과 로봇팔 제어를 시작합니다.
3. YOLOv8n과 RealSense L515의 RGB·깊이 영상으로 상자의 종류와 위치를 구합니다. xArm5가 빨강 → 노랑 → 파랑 순서로 상자를 파지해 차량 측면 트레이에 적재합니다.

실내 시연에서 픽업존 접근·정지와 세 상자의 순차 적재를 확인했습니다.

## 코드 구성

| 경로 | 역할 |
|---|---|
| `local_ws/src/scout_dwa` | 센서·지도 연동, 주행 연결, 명령 감시 |
| `local_ws/src/dwa_planner` | DWA 주행 후보 생성과 평가 |
| `local_ws/src/use_yolo` | YOLOv8n 검출, xArm5 파지·적재 제어 |
| `launch/system.launch` | 차량·센서·ROS 환경 준비 |
| `run_dwa_then_xarm.py` | 주행에서 인식·파지 단계로 전환 |

실내 주행 설정에서 DWA 도착 허용 거리는 **0.3 m**입니다. 목표점 주변 0.75 m의 장애물 입력은 DWA 계획 단계에서 제외하도록 구성되어 있습니다. 두 값은 코드 설정값입니다.

## 실행

ROS 1 Noetic과 Scout·Velodyne·HDL Localization·RealSense·xArm 드라이버, MoveIt이 준비된 Linux 환경이 필요합니다. 의존성 버전은 [dependencies.repos](dependencies.repos), 패키지별 출처는 [THIRD_PARTY.md](docs/THIRD_PARTY.md)를 참고하세요.

저장소 루트에서 준비 환경을 실행합니다.

```bash
bash start_all.sh --map-pcd "$PWD/maps/m304.pcd"
```

RViz의 **2D Pose Estimate**로 초기 위치와 방향을 입력한 뒤, 같은 저장소의 별도 터미널에서 실행합니다.

```bash
source local_ws/devel/setup.bash
python3 run_dwa_then_xarm.py --run --map-pcd "$PWD/maps/m304.pcd"
```

목표 대기 메시지가 표시되면 RViz의 **2D Nav Goal**로 픽업존을 지정합니다. 기반 워크스페이스가 기본 경로와 다르면 `SCOUT_BASE_SETUP`에 해당 환경의 `devel/setup.bash`를 지정합니다.

[시스템 구성](docs/SYSTEM_DESIGN.md) · [통합 중 문제 해결](docs/INTEGRATION_DEBUGGING.md)
