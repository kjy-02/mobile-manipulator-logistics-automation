# 사용한 HDL 패키지 스냅샷

졸업작품의 실제 작업 환경에서 가져온 세 패키지다. 원래 소스·설정과 기존 라이선스 파일을 보존하고 Git 관리용 내부 디렉터리는 제외했다.

| 패키지 | 출처 | 원본 ZIP의 Git HEAD |
|---|---|---|
| hdl_graph_slam | https://github.com/koide3/hdl_graph_slam | 95b8dce41c10667dcf0e3e9801dc22754b3bd525 |
| hdl_localization | https://github.com/koide3/hdl_localization | de2dc7769aa2412876d05ae6161408d3295b6a3c |
| hdl_global_localization | https://github.com/koide3/hdl_global_localization | a6a6b69929ac371d9531d0dcdef3c1593fd49245 |

로컬 수정이 포함될 수 있어 위 커밋만 내려받은 내용과 완전히 같다고 가정하지 않는다.

별도의 기반 catkin workspace의 src에 필요한 패키지 폴더를 복사하고, 각 패키지의 README에 따라 g2o·ndt_omp·fast_gicp 등 의존성을 준비한다. `dependencies.repos`로 같은 이름의 HDL 패키지를 중복으로 가져오지 않도록 한다. 기존 로봇 환경에서는 정상 동작하던 기반 workspace를 우선 사용한다.

`hdl_graph_slam`은 사전 지도 작성에 사용한다. 저장된 PCD 기반의 최종 주행은 `scout_dwa`의 지도·위치 추정 launch에서 실행한다. 개별 HDL launch의 지도 경로·TF·센서 높이 등은 실험 당시 값이므로 통합 launch 설정과 구분한다.

이 스냅샷을 포함한 새 PC 전체 환경 빌드는 이번 업로드 과정에서 실행하지 않았다.
