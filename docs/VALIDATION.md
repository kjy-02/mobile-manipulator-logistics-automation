# 배포본 빌드 및 검증

## GitHub 공개본 점검 (2026-10-29)

- 공개본에는 기존 실행 소스와 프로젝트 소개, 통합 시험 문제 해결 문서, 사진, HDL 패키지 3개의 소스 사본을 포함했다.
- 공개본을 만드는 과정에서 프로젝트 코드의 계산과 제어 로직은 바꾸지 않았다.
- Python 43개 파일의 구문을 점검했다. 42개는 Python 3 구문 검사 통과, 외부 HDL의 과거 데이터셋 변환 유틸리티 `ford2bag.py` 1개는 Python 2 문법이므로 Python 3에서 직접 실행할 수 없다. 해당 도구는 현재 주행·파지 실행 경로에 사용하지 않는다.
- 위 Python 43개 파일 검사와 기존 Python·launch/XML 84개 구문 검사는 검사 대상이 달라 각각 기록했다.
- `SHA256SUMS`에 GitHub 공개본의 파일 해시를 기록했다. 해시 목록 자체, 로컬 전용 문서와 백업은 제외했다.
- 720p 시연 영상은 `demo-v1` 릴리스에 첨부했다. 고화질 원본은 별도로 보관했다.
- Windows에서는 공개본의 구문과 파일 구성을 점검했다. ROS 빌드와 실차 재시험은 이 점검에서 수행하지 않았다. Linux 빌드는 아래 단계에서 진행했다.

## Linux 빌드 및 오프라인 점검 (2026-10-27)

- 배포 폴더의 세 패키지 전체를 `catkin_make -j2`로 빌드: 성공.
- 기반 환경: 현재 PC의 `/home/rilab/catkin_ws/devel/setup.bash`.
- 빌드 결과와 devel은 `/tmp/scout_export_build`, `/tmp/scout_export_devel`에 생성. 배포 폴더에는 포함하지 않음.
- Python 파일 문법, launch XML 문법, 준비 shell 구문 확인: 통과.
- 두 루트 Python 실행기의 `--help` 실행: 통과.
- 전체 launch를 `roslaunch --dump-params`로 오프라인 해석: 성공.
- 당시 전체 launch의 기본 지도는 `maps/outdoor.pcd`였다. 현재 실내 실행 예시는 `maps/m304.pcd`를 지정한다.
- RealSense `filters` 빈 값, 외부 guard 파라미터 없음 확인.
- 실행용 원본과 비교하여 xArm·YOLO 코드, ONNX 모델, DWA C++ 알고리즘 파일이 바이트 단위로 동일함을 확인.
- 패키지 소스 내용의 배포용 차이는 두 지도 기본 경로뿐임을 확인. catkin 최상위 CMake 파일은 기존 시스템 심볼릭 링크의 내용을 일반 파일로 복사.
- 기준 원본과 일치하는 패키지 파일은 159개였다.

위 항목은 차량·팔·카메라를 구동하거나 ROS master에 이동 명령을 보내지 않고 확인했다. 배포 폴더의 실차 통합 시험과 새 PC에서의 의존성 설치·실행은 이 점검에 포함되지 않았다.

별도로 진행한 실차 시험과 통합 시연은 [통합 시험 문제 해결](INTEGRATION_DEBUGGING.md)에 정리했다.
