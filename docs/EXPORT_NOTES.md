# 배포본 작성 기록

GitHub 공개본에는 2026-10-29에 프로젝트 소개, 기술 문서, 사진과 HDL 패키지 소스 사본을 추가하고 `SHA256SUMS`를 갱신했다. 아래는 10월 27일 최초 배포 기록이다. 당시 ZIP과 추출 해시는 별도로 보관했다.

- 작성일: 2026-10-27
- 원본 작업 공간: `/home/rilab/catkin_ws/src/2026scout`
- 배포 폴더: `github_export/scout_dwa_xarm`
- 복사 대상: 현재 실행용 `local_ws/src/scout_dwa`, `dwa_planner`, `use_yolo` 및 루트 실행 스크립트와 launch.
- 포함 데이터: 현재 ONNX 모델, 야외 `outdoor.pcd`, 3층 `floor3.pcd`, 하드실 `hard.pcd`.
- 제외 항목: 빌드 산출물(`build`, `devel`, `install`), Python 캐시, 이전 ZIP과 rosbag, 작업용 백업, 개발 도구의 로컬 지침과 설정.

## 배포본에만 적용한 경로 변경

1. `start_all.sh`: 기반 환경 `/home/rilab/...` 고정 경로 대신 `$HOME/catkin_ws/devel/setup.bash`를 기본값으로 사용. `SCOUT_BASE_SETUP`으로 다른 경로 지정 가능.
2. 최초 배포 때 `start_all.py`의 기본 지도는 배포 폴더의 `maps/outdoor.pcd`였다. 현재 기본 지도 파일명은 `m304.pcd`다.
3. 최초 배포 때 `run_goal_navigation.py`도 배포 폴더의 `maps/outdoor.pcd`에서 지도를 확인했다. 현재 기본 지도 파일명은 `m304.pcd`다.
4. 최초 배포 때 전체 launch와 지도 launch의 기본 지도 경로를 배포 폴더 기준 상대 경로로 지정했다. 현재 실행 예시는 `--map-pcd`에 저장소의 `maps/m304.pcd`를 넘긴다.
5. `local_ws/src/CMakeLists.txt`: 시스템 절대 심볼릭 링크 대신 catkin 최상위 파일을 포함.

최초 배포 작업은 배포용 사본에만 적용했다. DWA 알고리즘, xArm 동작, YOLO 검출 코드와 모델, 센서 TF, 장비 IP, 속도 설정은 실행용 원본과 같았다.

## 파일 확인

배포 당시 파일을 기록된 해시와 대조하려면 저장소 루트에서 다음 명령을 실행한다.

```bash
sha256sum -c SHA256SUMS
```

공개 이후 수정된 파일은 배포 당시 해시와 다를 수 있다. 빌드와 구문 검사 결과는 [배포본 빌드 및 검증](VALIDATION.md)에 정리했다.
