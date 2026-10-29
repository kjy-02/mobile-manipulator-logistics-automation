# 배포본 작성 기록

- 작성일: 2026-10-27
- 원본 작업 공간: `/home/rilab/catkin_ws/src/2026scout`
- 배포 폴더: `github_export/scout_dwa_xarm`
- 복사 대상: 현재 실행용 `local_ws/src/scout_dwa`, `dwa_planner`, `use_yolo` 및 루트 실행 스크립트와 launch.
- 포함 데이터: 현재 ONNX 모델, 야외 `outdoor.pcd`, 3층 `floor3.pcd`, 하드실 `hard.pcd`.
- 제외: 빌드·devel·install, Python 캐시, 과거 zip·bag, 작업용 백업, 에이전트 지침·설정.

## 배포본에만 적용한 경로 변경

1. `start_all.sh`: 기반 환경 `/home/rilab/...` 고정 경로 대신 `$HOME/catkin_ws/devel/setup.bash`를 기본값으로 사용. `SCOUT_BASE_SETUP`으로 다른 경로 지정 가능.
2. `start_all.py`: 기본 지도를 배포 폴더의 `maps/outdoor.pcd`로 지정.
3. `run_goal_navigation.py`: 기본 지도 확인 경로를 배포 폴더의 `maps/outdoor.pcd`로 지정.
4. 전체 launch와 지도 launch: 기본 지도 경로를 배포 폴더 기준 상대 경로로 지정.
5. `local_ws/src/CMakeLists.txt`: 시스템 절대 심볼릭 링크 대신 catkin 최상위 파일을 포함.

DWA 알고리즘, xArm 동작, YOLO 검출, 모델, 센서 TF·장비 IP·속도 설정은 배포 편집으로 바꾸지 않았다. 현재 운용 작업 공간의 파일은 수정하지 않았다.

## 파일 확인

루트에서 다음 명령으로 최초 배포 파일을 확인할 수 있다.

```bash
sha256sum -c SHA256SUMS
```

코드를 나중에 수정하면 해당 파일 해시가 달라진다. 검증 결과는 `VALIDATION.md`를 확인한다.
