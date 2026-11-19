## 2026-10-03: 도착 위치 허용 오차 50cm (사용자 요청)

- config/dwa_scout_low_speed_test.yaml GOAL_THRESHOLD: 0.2 → 0.5 m.
- scripts/run_goal_navigation.py arrived(): 거리 기준 0.2 → 0.5 m. 방향 기준 0.1 rad와 finish 신호 확인 유지.
- RUN_GOAL_NAVIGATION.md 갱신. YAML/Python 문법 및 두 거리 기준 일치 확인. 실제 주행 및 실행 중 파라미터 변경은 수행하지 않음.

## 2026-10-03: 제자리 회전 속도 1.5배 (사용자 요청)

- config/dwa_scout_low_speed_test.yaml: MAX_IN_PLACE_YAWRATE 0.6 → 0.9 rad/s, MIN_IN_PLACE_YAWRATE 0.3 → 0.45 rad/s.
- 전진·일반 회전·가속도 설정과 외부 guard 유지. 기존 회전 출력 한도 1.0 rad/s 안이므로 출력 제한 변경 없음.
- RUN_GOAL_NAVIGATION.md 현재 값 갱신. YAML 설정 확인 완료. 실제 주행은 실행하지 않음.

## 2026-10-03: 원본 속도 설정 적용 (사용자 요청)

- 원본 DWA 실행 유지. 속도·가속도 관련 11개 설정을 원본 GitHub와 동일하게 변경.
- 목표 선속도 0.55 m/s, 최대 선속도 설정 1.0 m/s, 최대 주행 회전 1.0 rad/s, 최대 제자리 회전 0.6 rad/s.
- 출력 guard·mission·watchdog의 속도 제한을 맞춤. 변경 파일: config/dwa_scout_low_speed_test.yaml, launch/goal_navigation.launch, scripts/run_goal_navigation.py, scripts/command_watchdog.py, RUN_GOAL_NAVIGATION.md, 본 문서.
- 기존 값은 config/speed_backups/20261003_192039_before_original_speeds에 비실행 .bak 파일로 보관.
- 검증: 원본 11개 값과 동일성, Python/XML 문법 확인. 실제 주행은 실행하지 않음. 적용하려면 주행 연결과 mission 재실행 필요.

## 2026-10-03: 사용자 요청으로 원본 DWA 실행 복원

- 목표 주행은 `low_speed_planner.launch`를 통해 원본 `dwa_planner`를 실행한다. 개선판 소스는 보존하지만 현재 mission에서는 실행하지 않는다.
- 원본 커밋 `6387a9660c2064fa0545ac81d55f4bbe7962651f`, git 작업 트리 변경 없음 확인.
- 변경: `scripts/run_goal_navigation.py`, `launch/low_speed_planner.launch`, `README.md`, `RUN_GOAL_NAVIGATION.md`, 본 문서.
- 원본의 비용 정규화, 목표·장애물 비용, 후보 생성, 지도 추출 및 충돌 검사를 사용한다. 별도 조향 비용은 비활성이다.
- 환경 연결·외부 guard 차체 경로 검사·watchdog·상태 감시·단일 목표 종료 기능은 유지한다. 전체 시스템이 원본과 동일한 구성이라는 뜻은 아니다.
- 목표 속도 0.30 m/s, 주행 회전 상한 0.65 rad/s 유지. 제자리 회전 상한은 0.32727 rad/s.
- 검증: Python/XML 문법, 실행 대상, 원본 실행 파일 존재 및 소스 동일성 확인. 실차 주행은 수행하지 않음.
- 남은 조건: 원본 복원 후 회피·도착 결과 확인, 외부 guard 정지 사유와 원본 DWA 정지 선택을 구분하여 해석.

# AMSL DWA 구현 및 실물 연결 상태

## 2026-10-03: 전체 격자 충돌 검사·조향 비용·차체 경로 guard 개선

사용자 승인 범위 1~3번을 적용했다. 현재 목표 주행은 `scout_dwa` 내 별도 개선판을 사용한다.
원본 고정 commit 저장소는 수정하지 않았고 SHA256 검증 통과.

변경 파일: `src/improved_dwa/`(원본에서 파생, 별도 클래스·실행 파일·LICENSE), `src/swept_footprint.h`, `src/cmd_vel_guard.cpp`, `launch/improved_planner.launch`, `launch/goal_navigation.launch`, `launch/first_motion_checks.launch`, `scripts/run_goal_navigation.py`, `CMakeLists.txt`, README·실행 문서. 검증 코드: `scripts/compare_saved_dwa.py`, `test/test_swept_footprint.cpp`.

1. 전체 occupied cell + 셀 면적 + 중간 차체 경로를 보수적으로 검사. 예측 충돌 후보는 제외하며 최종 yaw에도 적용.
2. 후보별 min/max 정규화를 제거하고 고정 척도 비용·조향량·조향 변화 비용을 사용. 저속 회전 강제값을 제거하고 직진 후보를 유지.
3. 목표 주행에서 고정 0.61m 원형 veto 및 별도 회전 원형 veto를 제거. 전체 local map에 실제 직사각형 footprint의 명령·측정 속도 진행 경로 guard 적용. 다른 legacy launch는 기존 기본 동작 유지.

검증: 개선 planner·guard 빌드 성공. 6개 차체 경로 시나리오 통과(원 안의 옆 장애물에서 안전한 직진 허용, 좌측 전방 장애물 접근 회전 차단·반대 회전 허용, 샘플 사이 장애물·제자리 회전 영역 검사).
저장 입력 비교는 실차 master와 분리한 11531 포트에서 수행했다. 정지 입력의 원본 `(v,w)=(0.01,+0.05)`에서 개선판 `(0.02,0)`으로 변경됐다. 가정 v=0.24 입력에서는 원본 42/63 후보 통과에서 개선판 21/63으로 변경됐다. 개선판은 이 경우에도 안전 여유 비용으로 약한 좌회전을 선택하므로 모든 좌회전을 금지했다고 주장하지 않는다.
기록: `validation/2026-10-03/pillar_incident/improvement_comparison.json`, baseline/improved offline 로그. 앞서 전체 격점과 겹쳤지만 원본이 허용하던 후보 index 21·22는 개선판에서 둘 다 제외됨을 직접 확인했다. 분석용 planner와 별도 master는 종료했다.

남은 조건: 개선판 실차 주행·전체 목표 도착은 미검증. 목표는 RViz 로그의 소수점 3자리 복원값이며 사고 순간 입력 재생은 아니다. 측면을 포함한 full-map 차체 보호는 기존 ±60° guard 범위를 변경한다. guard의 0.3초 반응·1m/s² 가정 감속도는 실측값이 아니다. 전역 계획은 추가하지 않았다. 현재 실행 중인 실차 launch를 재시작하지 않았으며 새 바이너리는 사용자 재시작 후 적용된다.

갱신: 2026-09-30. 원본 commit `6387a9660c2064fa0545ac81d55f4bbe7962651f`의 checkout과 핵심 파일 SHA256 검증 통과. 원본은 수정하지 않았다.

## 최신 변경: 멀티3층 지도 및 단일 목표 연속 주행

### guard 활성화 직후 disabled 종료 수정

추가 사용자 지시: 목표 주행 launch의 localization 품질 검사 및 좌표 급변 영구 차단을 비활성화했다. 변경: `scripts/localization_monitor.py`, `launch/first_motion_checks.launch`, `launch/goal_navigation.launch`. 상태 입력의 시각·연속 수신·timeout 검사는 유지한다. 원본 알고리즘 변경 없음. 테스트·차량 실행은 수행하지 않았다. 실행 중인 goal_navigation launch를 재시작해야 적용된다.

- 사용자 로그에서 활성화 직후 이전 disabled 상태를 읽고 즉시 종료하는 흐름 확인.
- `scripts/run_goal_navigation.py`: 활성화 서비스 성공을 확인하고 응답 이후 새 command/ready/reason 수신까지 0 속도로 대기. disabled가 아닌 새 상태 수신 후 기존 주행 검사로 진입하며, 실제 주행은 기존 모든 조건이 충족될 때만 시작한다. 상태 갱신 대기는 최대 3초이며 실패하면 정지·종료한다.
- 사용자 요청에 따라 코드 수정만 수행했다. 테스트·차량 실행은 하지 않았다. Python 실행 파일 변경으로 기존 rosrun 명령에 반영된다.

- 사용자 정정 경로 `/home/rilab/catkin_ws/map/my_map.pcd` 파일 존재 확인(13,537,422 bytes). 지도 이름 **멀티3층**.
- `launch/hdl_localization_map.launch` 기본 경로 수정. 이전 하드실 HDL·기존 DWA·first_motion_checks 실행을 종료하고 새 지도 HDL 재시작. 새 지도 초기 위치는 지정하지 않았다.
- 연속 목표 주행 연결: `scripts/run_goal_navigation.py`, `scripts/command_watchdog.py`, `launch/goal_navigation.launch`, `launch/low_speed_planner.launch`, `launch/first_motion_checks.launch`.
- 원본 planner를 새 목표마다 시작하고, 원본 완료 신호 및 map 위치·방향을 확인한 뒤 도착 로그·0 속도·guard 해제·자체 planner 종료. 2초 시험 제한 대신 사용자 설정 최대 시간까지 주행한다.
- DWA는 전체 각도 local map, 이동 근접 정지는 ±60° front_obstacles. 제자리 회전은 전체 local map의 footprint 외접 영역을 외부 코드에서 확인한다.
- 기타 변경: `CMakeLists.txt`, `package.xml`, `config/field_observed.yaml`, `rviz/scout_dwa.rviz`, `README.md`, `RUN_GOAL_NAVIGATION.md`. 과거 하드실 관측 수치는 멀티3층 실측으로 재사용하지 않는다.
- 검증: Python 구문 검사 통과, goal_navigation launch parameter dump 통과, adapter/guard/footprint 빌드 성공 및 새 Python 실행 wrapper 생성 확인. 원본 commit·SHA256 동일성 검증 통과.
- 사용자가 추가 시험을 중단하고 직접 실행·관찰하겠다고 지시하여 이후 합성 시험·실차 목표 주행을 진행하지 않았다.
- 남은 조건: 새 지도 초기 위치와 정합, 새 목표 입력, 전체 목표 도착·최종 회전·실제 정지 성능 현장 확인. 새 mission/watchdog는 아직 실차에서 실행하지 않았다.
- 실제 수동 실행 명령 및 로그 위치는 `RUN_GOAL_NAVIGATION.md` 참조.

## 현재 작업 범위

사용자의 최신 지시에 따라 **원본 저장소 구조를 그대로 쓰고 실물 연결에 필요한 부분만 맞춘다.**
기본 연결은 `launch/scout_dwa.launch`이며 원본 `dwa_planner/launch/local_planner.launch`를 직접 include한다.
추가 시험 확장은 중단했다. 이미 작성한 gateway/preflight/session 코드는 선택 가능한 확장 구성에 보존한다.

## 현재 실물 확인 결과

- 이 컴퓨터가 실제 Scout에 연결된 컴퓨터임을 사용자가 확인했다.
- LiDAR 장치 CGI 응답: **Velodyne VLP-16-A**, IP **192.168.1.201**, UDP 2368 실제 패킷 수신.
- ROS master와 VLP16 수신 노드만 시작했다. 실제 `/velodyne_points`, frame `velodyne`, 약 10 Hz 확인.
- `can0` **UP**, bitrate **500000** 확인. 차량 상태 프레임 `0x211`, `0x221`, `0x241` 수신으로 SDK 기준 **AGX V2 통신** 확인. 통신 버전만으로 Scout 모델을 확정하지 않는다.
- Scout 제어 노드, wheel odometry, localization, TF는 실행되지 않았다.
- Scout 모델은 기존 launch의 V2 xacro와 Mini 설정이 혼재하여 미확정이다.
- 사용자 설명상 Scout 2.0이 후보다. 기존 bringup의 Mini 인자는 하위 launch에 전달되지 않으므로 실제 드라이버 기본값은 false다.
- 실제 속도 명령, CAN 송신, 차량 구동은 수행하지 않았다.
- 관측 기록: `config/field_observed.yaml`, `validation/2026-09-30/live_preflight.json`.

## 단계별 변경·검증·남은 조건

| 단계 | 구현 및 변경 파일 | 검증 결과 | 남은 조건 |
|---|---|---|---|
| P0 | `scout_dwa_noetic/CATKIN_IGNORE`, 원본 검증 script | 고정 commit/clean/SHA256 통과 | 비교본을 실행 원본으로 사용하지 않음 |
| P1 | `launch/scout_dwa.launch`, `scripts/input_gateway.py`, `scripts/planner_preflight.py`, validator | 원본 launch 최소 연결의 parameter dump, custom frame·disabled output·새 세션 시험, 설정 단위시험 통과 | 현장 토픽·모델에 맞게 설정; 확장 gateway는 기본 연결에 필수 아님 |
| P2 | `src/cloud_obstacle_adapter.cpp`, sensor health/diagnostics | layout/offset/크기/빈값/NaN/시각/TF 오류, organized padding, 중심선, translation+rotation map/scan 시험 통과. 실물 raw cloud 수신 확인 | base-to-LiDAR 실측 TF, 높이·self filter·coverage 확인 |
| P3 | localization monitor/session, HDL status remap | 품질·시각·중단·inhibit·명시적 재검사와 map→odom jump 시험 통과 | 실제 HDL 초기화·TF chain·품질 임계값 |
| P4 | `src/cmd_vel_guard.cpp`, `src/footprint_validation.h` | 22개 독립 fault, disable/recovery, 외접원 veto, guard SIGTERM/SIGKILL + 합성 하위 watchdog 통과 | 실측 footprint·정지 조건, 실제 driver/E-stop/watchdog 확인 |
| P5 | `test/`의 원본 기능·고장·세션 시험 | map/scan, 전체 후보 충돌의 지속 zero, circle/polygon, yaw/finish, path cost 선택, markers, thin scan 누락 재현 및 현재 footprint veto 통과 | future-path 얇은 장애물 누락은 원본 한계로 남음; 실차 주행 미검증 |
| P6 | README, 파라미터 출처표, RViz, pending/observed YAML, 로그 | 독립 두 패키지 Release 빌드, 현재 catkin 워크스페이스 빌드 성공 | 실물 연결과 확인값 반영 |
| P7 | 실물 LiDAR·CAN 읽기 전용 확인, observed YAML | VLP16 센서와 500000 bitrate CAN 상태 프레임 실수신 확인 | Scout 모델, wheel odom/TF, HDL 연결 및 별도 주행 지시 |

## 검증 집계와 기록

- 독립 워크스페이스의 전체 합성시험: **42 tests, 0 errors, 0 failures**.
  당시 18개 rostest 시나리오, 5개 config 단위시험과 localization monitor의 추가 메서드가 포함됐다.
- 이후 추가한 map→odom jump 시나리오: **2 tests, 0 errors, 0 failures**.
  두 실행의 집계를 단일 full rerun으로 표현하지 않는다.
- rostest wrapper와 내부 rosunit은 이중 집계된다. 제약 재현 성공은 실차 안전성 성공을 뜻하지 않는다.
- 현재 워크스페이스 명령: `catkin_make --pkg scout_dwa dwa_planner -j2 -DCMAKE_BUILD_TYPE=Release` 성공.
- 최소 launch의 실제 `/cmd_vel` 출력은 연결하지 않았으며 기본 guard는 disabled다.
- 로그: `validation/2026-09-30/`.

## 다음 작업

최신 실행 구성: DWA는 전체 각도 `/scout_dwa/local_map`, 근접 정지는 ±60도
`/scout_dwa/front_obstacles` 사용. 사용자 '추가 점검 없이 주행' 지시에 따라
별도 검증 확장 없이 제한 시험을 실행했다.
한도 불일치로 첫 0.412초에 중단(실측 이동 약0.0335m)되어 원본 planner를
`low_speed_planner.launch`로 재시작, v≤0.08,w≤0.15 소프트웨어 설정을 적용했다.
기존 목표는 RViz 로그의 소수3자리 기록(2.100,1.548,yaw0.050)으로 재적용했으며
원래 메시지 전체 정밀도를 보존한 것으로 표현하지 않는다.
후속 시험: 1.681초, wheel 이동 약0.1086m 후 지도/health 시각 시차로 정지.
시차 동안 zero 유지하고 같은2초 예산 내 재일치 시에만 전달하도록 bounded relay 수정.
마지막 시험: 1.642초, 약0.0720m 이동 후 제자리 회전 coverage 미확정 gate로 정지;
nonzero79개, 순간 시차 zero1회, 최종 v=0,w=0, guard disabled.
회전 coverage 미확정 gate는 해제하지 않았다. 정상 연속 자율주행은 아직 검증하지 않았다.
사용자 수동 실행용 `low_speed_field_test.launch`를 추가해 footprint·감시·원본 저속 planner를
묶었다. 현재 실제 실행 중인 노드는 유지했고 이 새 launch를 중복 기동하지 않았다.

첫 제한 실차 명령 시험 (사용자 반복 진행·안전 확인 후):
`launch/first_motion_checks.launch`는 원본 planner를 유지하고 외부 adapter·guard와
localization monitor만 연결한다. v≤0.1 m/s,w≤0.2 rad/s, 최대2초.
사용자 승인 장착값·footprint·실물 장애물 표시 및 현장 안전 확인은 이 짧은 attended
시험의 profile 승인으로만 적용했다(accepted_trial_*). 전체 coverage·hardware watchdog·
E-stop 실측 확인과 정상 자율주행 승인은 여전히 미확정이며 pending 값을 확정하지 않았다.
monitor rejection threshold error0.05,inlier0.9,pose jump0.1m/0.1rad는 실험적 소프트웨어 한도다.
기존 TF chain에 odom TF가 없으므로 jump는 map→base_link의 실제 pose 변화(운동 포함)를
검사한다. map→odom correction-only 검증으로 표현하지 않는다.
실물 TF 예외 처리에서 tf.TransformException 미존재를 발견해 tf.Exception으로 수정했고
status와 exact-stamped TF 도착 시차에는 최대0.1초 wait를 추가했다. input_gateway 동일 수정.
launch dump·Python syntax·원본 SHA 검증 통과. 실제 guard ready 1초 연속 확인 후 시험.
결과: nonzero vehicle 명령 8개, 약0.163초 후 footprint 외접원 occupied로 중단,
1초 zero 전송, 최종 fresh wheel odom v=0,w=0. guard 재차단 완료.
실제 이동량/최대 측정 속도는 시험 script가 기록하지 않아 실차 이동 검증 성공으로 선언하지 않는다.
2초 주행·비상정지·firmware watchdog 시험도 완료되지 않았다.
기록: validation/2026-09-30/first_motion_result.log. 다음은 근접점 원인 확인이다.
변경 파일: first_motion_checks.launch, scripts/localization_monitor.py,
scripts/input_gateway.py, 본 문서; 원본 소스 변경 없음.

사용자가 오른쪽 앞 물체를 치운 후 지도 5개 재확인: near cell 0개, 최근접 약1.675 m.
`scripts/bounded_drive_test.py`를 추가했다. 기본 모드는 관측만 수행하며 --run에서도
기존 guard ready·fresh 명령/odom·0.1 m/s,0.2 rad/s 한도·기존 publisher 없음·시작 시 정지를
확인한다. 원본 guarded 명령을 그대로 50 Hz로 최대2초 전달하고 정상 종료·예외·SIGINT/
SIGTERM 시 1초간 zero를 전송한다. guard를 enable하거나 미확인 gate를 승인하지 않는다.
SIGKILL/통신 고장의 정지는 차량 watchdog/E-stop이 필요하며 아직 미검증이다.
Python 구문 검사 및 실제 관측 모드 실행 통과. 현재 disabled여서 nonzero 명령 없음.
변경: 위 script, CMake install 목록, 본 문서. readiness JSON은 validation 폴더에 기록.

사용자 '진행해 안전해'로 저속 실차 시험 진행·주변 안전 확인 의사를 받았다.
직전 실물 점검: wheel 속도 0, localization 수렴 true(error 약 0.009, inlier 약0.998),
raw DWA v=0.05 m/s,w=0.15 rad/s. 다만 guard 주변에 점유 점 4개 이상,
최근접 약0.57~0.59 m: base_link x≈0.3 m,y≈-0.5 m(오른쪽 앞).
물체·사람 여부 및 정리 확인을 요청했다. 정지 조건을 완화하지 않았고 nonzero 차량
명령은 전송하지 않았다. 기록: validation/2026-09-30/motion_readiness.json.
이 현장 확인은 위치추정 health/coverage 미확정이나 watchdog 미검증을 승인 처리하지 않는다.

저속 시험 진행 지시 후 준비: 실제 `/cmd_vel` 기존 발행자 없음을 확인하고 0 속도만
1초, 50 Hz로 전송해 제어 연결을 확인했다. nonzero 명령은 아직 전송하지 않았다.
`config/dwa_scout_low_speed_test.yaml`에 첫 시험 소프트웨어 한도 v≤0.1 m/s,
|w|≤0.2 rad/s를 준비했다. 이것은 측정 하드웨어 한도가 아니다. launch dump 검사 통과,
고정 원본 SHA 검증 통과. 기록: validation/2026-09-30/zero_command_check.json.
드라이버 ROS callback에는 명령 끊김 시 zero 처리 없음; SDK는 callback당 CAN 송신이며
차량 firmware watchdog 정지는 아직 실물 미검증. zero 시험은 watchdog 검증을 대신하지 않는다.
사용자에게 주변 공간·비상정지 동작 및 즉시 사용 가능 상태 확인을 요청했다.
움직임 전 localization health·coverage gate 미확정을 해결해야 하며 이를 우회하지 않았다.

사용자 실시간 장애물 확인 요청: `rviz/obstacle_check.rviz` 생성, RViz만 교체.
원시 `/velodyne_points` 파랑과 `/scout_dwa/local_map` 점유 검정을 함께 표시하고
차체 footprint·축을 추가했다. 기존 하드실 지도는 선택 display로 보존.
센서 필터·원본 DWA·차량 출력 변경 없음. 장애물 감지 확인은 계속 미완료.

사용자 전방 장애물 설치 후 관측: base_link x=0.5~1.5 m, |y|≤0.5 m 구간
local map occupied cell 0개, 높이 필터 통과 raw point 0개. 이어 raw cloud 3프레임에서
모든 높이를 검사해도 이 구간 점 0개여서 높이 필터만의 문제라고 결론내리지 않았다.
최근접 occupied 점은 약 0.933 m, raw 명령은 계속 nonzero지만 실제 cmd_vel 발행자 없음.
기록: validation/2026-09-30/front_obstacle_check.json. 장애물 감지 시험은 **미통과**이며
물체 위치·크기·LiDAR 시야를 확인해야 한다. 주행 출력은 차단 유지.

사용자 확인 반영: 기존 base→LiDAR (+0.35,0,0, 회전0) 설정 사용 승인,
0.925×0.6996 m footprint 확인을 pending/observed YAML에 출처와 함께 기록했다.
실물 map 관측: footprint 사각형 내부 occupied cell 0개, guard 외접원 반경 약
0.671 m 내부 21개, 최근접 점 약 0.443 m. 이 점이 실제 주변 물체·차체·바닥인지
아직 판별하지 않았으므로 필터나 정지 반경을 임의로 완화하지 않았다.
HDL 수렴 true 확인, guard disabled, 실제 cmd_vel 발행자 없음.
다음: 정지 상태에서 주변 점 원인을 확인하고 약 1 m 앞 테스트 장애물 감지 확인,
위치추정 health 및 실제 E-stop/watchdog 확인 후 별도 실주행 운용 지시.

지도 동시 표시 요청 반영: globalmap을 alpha 0.25로 다시 켜고, 표시 전용 marker 복사
노드에서 경로 z를 +1 m, 선택 선 0.07 m, lifetime 0.3 s로 설정했다. 이것은 RViz 표시만
조정하며 실제 DWA 경로·충돌 계산·제어 메시지에는 영향을 주지 않는다.
변경: scripts/trajectory_display.py, launch/visualization.launch, rviz 설정,
CMake Python install, package.xml visualization_msgs 의존성, 본 문서.
Python 구문 검사 통과. 지도와 표시 전용 경로 토픽 연결 확인; 원본 소스 보존.
다시 실행: `roslaunch scout_dwa visualization.launch`. 실차 출력 차단 유지.

경로 표시 누락 수정: RViz root Enabled/Value와 Displays·Views 패널 이름을 명시했다.
실제 화면 캡처로 표시 활성화 확인. 짧은 local trajectory가 점군·map에 가려지는 것을
피하도록 해당 배경 display 기본값을 off하고 카메라 확대했다. 필요 시 체크해 다시 볼 수 있다.
변경 파일: rviz/scout_dwa.rviz, 본 문서. 원본 계산·차량 출력 변경 없음.

경로 시각화 요청: `rviz/scout_dwa.rviz`에 하드실 globalmap·aligned LiDAR,
원본 후보 MarkerArray·선택 Marker·local map·footprint, 초기 pose·goal 도구와
base_link 중심 위에서 보는 카메라를 구성했다. 이전 RViz만 종료하고 이 설정으로
재실행했다. 원본 색상은 선택 빨강, 유효 후보 초록, 충돌 후보 보라다.
DWA·localization·Scout driver는 유지했고 차량 출력 상태는 변경하지 않았다.

목표 재지정 후 실물 입력 관측 성공: 5초 local map 50개, raw 명령 100개 모두 nonzero,
최근 linear.x=0.05 m/s, angular.z=0.10263 rad/s. selected trajectory 100개(10 points),
candidate marker array 100개(최근 126 markers), 최근 occupied grid cells 1066.
`/cmd_vel` 발행자 없음으로 차량 구동 출력과 분리됨을 재확인했다.
기록: `validation/2026-09-30/live_dwa_output.json`; 원본 알고리즘 변경 없음.
이 결과는 실물 입력의 계산 연결 성공이며 실주행 검증은 아니다. 센서 보정·footprint·
coverage 확인과 localization health 연결, 별도 운용 지시는 여전히 남아 있다.

RViz 목표 이후 raw 3개 샘플은 zero였다. 센서 TF의 동적 시각 조회 실패를 확인해
동일 -0.35 m 설정을 `launch/legacy_sensor_tf.launch`의 tf2 정적 TF로 전환했다.
기존 TF 스크립트만 종료하고 DWA 최소 세션을 재시작하여 기존 TF cache를 제거했다.
새 base_link local map 수신 확인. 목표 재지정 후 경로·속도 계산 재관측이 필요하다.
변경 파일: legacy_sensor_tf launch, package.xml(tf2_ros runtime), README, 본 문서.
기하 보정값은 여전히 미확정이며 원본 알고리즘 변경 없음. 차량 출력 차단 유지.

후続 연결: 사용자 '다음' 지시에 따라 `/cmd_vel`·light 제어 발행자가 없는지 확인 후
Scout driver를 Mini=false, Omni=false, pub_tf=false로 시작했다. SDK가 CAN 제어 모드를
전환했다(control_mode=1). wheel odom 5초 251개, 약 50 Hz, 속도 0, fault_code=0.
모델은 후보 설정으로 실행한 것이며 물리 모델 확인을 대신하지 않는다.
원본 DWA 최소 launch도 실제 입력으로 시작했다. base_link local map 120x120, 0.05 m
수신 확인; guard reason=disabled, 실제 `/cmd_vel` 발행자 없음. 목표는 지정하지 않았다.
기존 1 Hz 동적 extrinsic TF는 map 갱신 지연 경고를 유발할 수 있어 고정 TF 전환을
다음 연결 개선 항목으로 남긴다. 실제 주행은 수행하지 않았다.
변경 파일: observed YAML, 본 문서, live_preflight.json; 원본 소스 변경 없음.

하드실 초기 위치 사용자 지정 후 5초 관측: localization odom·status 각 50개,
수렴 true, 최근 matching_error 약 0.0105, inlier_fraction 약 0.9956.
`map -> velodyne` TF 수신 확인. 기존 `tf_code/velodyne_to_baselink.py`를 시작했다.
그 translation -0.35 m는 기존 설정이며 실측 승인값은 아니다.
Scout wheel odom은 여전히 없으며 driver·DWA·실차 속도 출력은 시작하지 않았다.
변경 파일: observed YAML, 본 문서, live_preflight.json 관측 결과.

지도 변경 (사용자 지정): 현재 **하드실**
`/home/rilab/catkin_ws/src/hdl_localization/data/map/hard.pcd`.
기존 `/home/rilab/catkin_ws/src/hdl_graph_slam/map.pcd`는 **야외 학생회관**으로 기록했다.
변경 파일: `launch/hdl_localization_map.launch`, `config/field_observed.yaml`, README, 본 문서.
기존 HDL 세션만 종료하고 새 지도로 재시작했다. 실행 파라미터에서 hard.pcd 경로 확인,
`/globalmap` frame `map`, downsampled 33,622 points 수신 확인. 새 지도 초기 pose 입력 필요.
RViz와 LiDAR는 유지했고 실제 차량 구동은 수행하지 않았다.

기존 실행 명령 반영: `dwa_map.launch`를 최소 `scout_dwa.launch`로 연결하고,
`scout_base_for_dwa.launch`에 `pub_tf` 인자를 추가했다. 기존 `map -> velodyne -> base_link`
TF를 유지할 때는 Scout `pub_tf:=false`로 중복 child TF를 방지한다. README에 실행 연결,
드라이버 시작 시 commanded mode 전환, waypoint 인자 미지원과 출력 차단 조건을 기록했다.
변경 파일: 위 두 launch, README, `config/field_observed.yaml`, 본 문서.
검증: 두 launch parameter dump 통과, 원본 고정 commit·SHA256 검증 통과.
후속 실물 점검에서 HDL localization을 시작했다. `/globalmap` publisher와 HDL 구독 연결을
확인했고 실제 cloud는 5초간 50개 수신했다. HDL은 `waiting for initial pose input` 상태다.
`/localization/odom`·status는 관측 기간 0개, TF edge 없음, Scout 드라이버와 wheel odom 없음.
`/cmd_vel` 토픽도 없어 차량 속도 명령은 발행되지 않았다. 초기 위치는 임의 지정하지 않았다.
`validation/2026-09-30/live_preflight.json`과 observed YAML을 갱신했다.

1. CAN 활성화·상태 프레임 수신 확인 완료.
2. 실제 Scout 상태와 wheel odom·TF를 확인하고 모델 설정 혼재를 해결한다.
3. 실물 geometry와 HDL 입력을 연결한 뒤 원본 DWA의 raw 출력만 확인한다.
4. 실제 주행은 별도 운용 지시 후 진행한다.

## 보존된 원본 한계

최종 yaw의 내부 충돌검사 부재, 회전 중 목표 교체 상태, 반복 finish flag, path 첫·끝 직선 비용,
scan subsampling 및 map ray 사이의 얇은 장애물 누락은 원본에 남는다. 외부 veto는 현재
footprint 안의 이동·회전을 차단하지만 앞으로 이동할 경로의 누락을 해결했다고 주장하지 않는다.
`sampling_limits.json`은 산술 재현 진단이며 실차 충돌검증 자료가 아니다.

## 2026-10-28 — 고정 목표 입력 추가

현재 실행용 `run_goal_navigation.py`에 `--goal X Y YAW`를 추가했다. X/Y는 map 기준 미터, yaw는 라디안이다. 옵션을 지정하면 odometry 수신 후 내부 PoseStamped를 생성해 기존 목표 처리 경로로 전달하며 RViz 목표 입력은 구독하지 않는다. 옵션 생략 시 기존 RViz 방식이다. 도착 후 xArm 실행 연결과 DWA 계산·속도·지도 설정은 유지했다. 좌표값은 아직 지정받지 않아 기본 목적지를 만들지 않았다. 고정 목표·yaw 변환·odometry 대기·수동 입력 유지·잘못된 인자 거부·상위 실행기의 인자 전달을 오프라인 확인했으며 실차는 실행하지 않았다. `github_export`의 기존 배포 스냅샷은 이번 변경에 맞춰 갱신하지 않았다.

## 2026-10-31 — 목표 도착 거리 0.3m

현재 통합 실행의 `config/dwa_scout_low_speed_test.yaml`에서 `GOAL_THRESHOLD`만 0.5에서 0.3으로 변경했다. `low_speed_planner.launch`의 실제 파라미터 전개에서 0.3 반영을 확인했고, 방향 기준 0.1rad·최소 속도 0.1m/s·로봇 반경은 유지했다. 통합 완료 판정은 기존 finish_flag만 사용하므로 별도 0.5m 조건 충돌은 없다. 현재 실행 경로에 거리 기준을 덮어쓰는 발행 코드는 포함되지 않지만 외부 실행 노드의 실시간 발행 여부는 오프라인 검사로 확인하지 않았다. 기존 has_reached_ 상태 때문에 거리 범위 진입 후 방향을 맞추는 동안에는 거리를 다시 검사하지 않는 점은 유지한다. 최종 정지 위치가 항상 반경 0.3m 안임을 보장하는 변경은 아니다. 다음 DWA 주행 프로세스 시작부터 적용되며 실차는 실행하지 않았다. 기존 GitHub 배포 스냅샷은 변경하지 않았다.

## 2026-10-31 — 도착 거리 0.15m로 축소

사용자 요청으로 현재 통합 실행 설정 `dwa_scout_low_speed_test.yaml`의 `GOAL_THRESHOLD`를 0.3에서 0.15m로 변경했다. launch 파라미터 전개에서 0.15를 확인했으며 방향 기준 0.1rad와 다른 설정은 유지했다. 다음 DWA 프로세스 시작부터 적용된다. 방향 정렬 중 거리 재검사를 하지 않는 기존 동작은 유지되며 실차 테스트는 수행하지 않았다.

## 2026-10-31 — 도착 거리 0.3m 복원

사용자 요청으로 `GOAL_THRESHOLD`를 0.15에서 0.3m로 복원했다. 방향 기준과 다른 설정은 변경하지 않았다. 다음 DWA 프로세스 시작부터 적용된다.

## 2026-10-31 — 목적지 중심 장애물 제외 반경 0.75m

사용자 요청으로 `GOAL_OBSTACLE_IGNORE_RADIUS: 0.75`를 현재 목표 주행 YAML에 추가했다. 기본값은 0.0(해제)이며 음수·NaN·무한대 설정은 해제 처리한다. 목표를 base_link로 변환한 뒤 원본 장애물 목록의 계획용 복사본에서 목표 중심 수평 반경 안의 점만 제외한다. 일반 궤적 장애물 비용과 회전 전 충돌 검사에 같은 목록을 사용한다. 센서·지도 원본, RViz 표시, 반경 밖 장애물 점은 유지한다. TF 변환 실패 시 잘못된 중심으로 계획하지 않도록 0 속도를 반환한다. 현재 도착 0.3m, 속도, 팔, watchdog은 변경하지 않았다. C++ 빌드와 필터 단위 테스트 및 실제 비용/충돌 함수 연결 검증을 통과했으며 실차는 실행하지 않았다. 백업은 `.dwa_update_backups/20261031_185813_110709`이고 GitHub 배포본은 이전 스냅샷 그대로다.

## 2026-10-31 — 목표 반경 필터의 장애물 추출 순서 수정

이전 구현은 방향별 첫 장애물을 추출한 후 반경 안의 점을 제거해, 그 뒤에 지도상으로 존재하던 반경 밖 벽이 누락됐다. 재현 지도에서 반경 밖 벽 24개 셀이 있어도 남은 장애물 목록이 비는 현상을 확인했다. local map 원본을 보관하고 매 계획 주기 목표 반경 안의 점을 먼저 건너뛰면서 광선을 계속 탐색하도록 수정했다. 반경 밖 첫 장애물은 유지하며, 원본 지도는 변경하지 않는다. 기능 해제 시 100개 지도의 샘플 좌표가 이전 추출 방식과 일치함을 검증했다. 기존 단위 테스트에 책상 뒤 벽 누락 회귀 사례를 추가했고 전체 빌드·launch 파라미터 검증을 통과했다. 제외 반경 0.75m, 도착 기준 0.3m, 속도·팔·watchdog은 유지했으며 실차는 실행하지 않았다. 물리적으로 관측되지 않아 지도에도 없는 장애물을 복원하는 기능은 아니다. 백업: `.dwa_update_backups/20261031_190847_525623`.
