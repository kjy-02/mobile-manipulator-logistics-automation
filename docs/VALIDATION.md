# 배포본 검증 결과 — 2026-10-27

- 배포 폴더의 세 패키지 전체를 `catkin_make -j2`로 빌드: 성공.
- 기반 환경: 현재 PC의 `/home/rilab/catkin_ws/devel/setup.bash`.
- 빌드 결과와 devel은 `/tmp/scout_export_build`, `/tmp/scout_export_devel`에 생성. 배포 폴더에는 포함하지 않음.
- Python 파일 문법, launch XML 문법, 준비 shell 구문 확인: 통과.
- 두 루트 Python 실행기의 `--help` 실행: 통과.
- 전체 launch를 `roslaunch --dump-params`로 오프라인 해석: 성공.
- 전체 launch 기본 지도: 배포 폴더의 `maps/outdoor.pcd`로 연결 확인.
- RealSense `filters` 빈 값, 외부 guard 파라미터 없음 확인.
- 실행용 원본과 비교하여 xArm·YOLO 코드, ONNX 모델, DWA C++ 알고리즘 파일이 바이트 단위로 동일함을 확인.
- 패키지 소스 내용의 배포용 차이는 두 지도 기본 경로뿐임을 확인. catkin 최상위 CMake 파일은 기존 시스템 심볼릭 링크의 내용을 일반 파일로 복사.
- 최종 기준 원본과 동일한 패키지 파일 159개 확인.

이 검증에서는 차량·팔·카메라를 실행하지 않았고 ROS master에 이동 명령을 보내지 않았다. 배포 폴더의 실차 통합 테스트 및 새 PC의 의존성 설치·실행은 수행하지 않았다.

현재 작업 공간에서 사용자가 확인한 정상 동작과, 위 배포본 오프라인 검증을 구분한다.
