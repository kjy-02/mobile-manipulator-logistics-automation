# 파라미터 출처와 확인 상태

2026-09-30. 고정 원본 YAML 두 개, 현재 reference YAML, Noetic 비교본 Scout YAML 두 개, synthetic YAML을 직접 대조했다. 아래 숫자는 실물 승인값이 아니다. 모든 현장 값은 pending이다.

| Parameter | 단위 | Pinned upstream YAML | Reference | Noetic 비교본 | Synthetic | 영향 | 현장 confirmed |
|---|---|---|---|---|---|---|---|
| TARGET_VELOCITY | m/s | 0.55 | 0.3 | 0.3 | 0.2 | 속도·범위·방향 | pending |
| PREDICT_TIME | s | 3 | 3 | 2 | 2 | 예측 시간·적분 간격 | pending |
| SIM_TIME_SAMPLES | — | 10 | 10 | 20 | 10 | 후보·예측 점 수 및 CPU | pending |
| SIM_PERIOD | s | 0.1 | 0.1 | 0.05 | 0.1 | 예측 시간·적분 간격 | pending |
| SIM_DIRECTION | rad | 1.57 | 1.57 | 1.57 | 1.57 | 속도·범위·방향 | pending |
| SLOW_VELOCITY_TH | m/s | 0.1 | 0.1 | 0.1 | 0.1 | 속도·범위·방향 | pending |
| VELOCITY_SAMPLES | — | 3 | 3 | 5 | 3 | 후보·예측 점 수 및 CPU | pending |
| YAWRATE_SAMPLES | — | 20 | 20 | 30 | 20 | 후보·예측 점 수 및 CPU | pending |
| OBSTACLE_COST_GAIN | — | 1 | 1 | 1 | 1 | 후보 비용 가중치 | pending |
| TO_GOAL_COST_GAIN | — | 0.8 | 0.8 | 0.8 | 0.8 | 후보 비용 가중치 | pending |
| SPEED_COST_GAIN | — | 0.4 | 0.4 | 0.4 | 0.4 | 후보 비용 가중치 | pending |
| PATH_COST_GAIN | — | 0.4 | 0.4 | 0 | 0.4 | 후보 비용 가중치 | pending |
| ANGLE_RESOLUTION | rad | 0.087 | 0.087 | 0.087 | 0.087 | 장애물 샘플링 간격 | pending |
| OBS_RANGE | m | 2.5 | 2.5 | 3 | 2.5 | 속도·범위·방향 | pending |
| GOAL_THRESHOLD | m | 0.1 | 0.2 | 0.5 | 0.1 | 목표·회전 전환 | pending |
| TURN_DIRECTION_THRESHOLD | rad | 0.1 | 0.1 | 3.2 | 0.1 | 목표·회전 전환 | pending |
| ANGLE_TO_GOAL_TH | rad | pi | 3.141592653589793 | 3.141592653589793 | 3.141592653589793 | 목표·회전 전환 | pending |
| ROBOT_FRAME | — | base_link | base_link | base_link | base_link | 좌표계 계약 | pending |
| ROBOT_RADIUS | m | 0.1 | 0.5 | 0.45 | 0.2 | 충돌 외곽 | pending |
| FOOTPRINT_PADDING | m | 0.01 | 0.02 | 0.05 | 0.01 | 충돌 외곽 | pending |
| MAX_VELOCITY | m/s | 1 | 0.5 | 0.5 | 0.4 | 속도·범위·방향 | pending |
| MIN_VELOCITY | m/s | 0 | 0 | 0 | 0 | 속도·범위·방향 | pending |
| MAX_YAWRATE | rad/s | 1 | 0.7 | 0.7 | 0.7 | 속도·범위·방향 | pending |
| MIN_YAWRATE | rad/s | 0.05 | 0.05 | 0.05 | 0.05 | 속도·범위·방향 | pending |
| MAX_IN_PLACE_YAWRATE | rad/s | 0.6 | 0.5 | 0.5 | 0.5 | 속도·범위·방향 | pending |
| MIN_IN_PLACE_YAWRATE | rad/s | 0.3 | 0.2 | 0.15 | 0.2 | 속도·범위·방향 | pending |
| MAX_ACCELERATION | m/s² | 0.5 | 0.5 | 0.3 | 0.5 | dynamic window 범위 | pending |
| MAX_DECELERATION | m/s² | 2 | 1 | 0.8 | 1 | 속도·범위·방향 | pending |
| MAX_D_YAWRATE | rad/s² | 3.2 | 1.5 | 1 | 1.5 | dynamic window 범위 | pending |

원본 `ANGLE_TO_GOAL_TH: pi`는 YAML 문자열이다. 환경 profile은 숫자 π를 전달하고 preflight가 실행에 쓰이는 숫자를 검증한다. HZ, GLOBAL_FRAME, ROBOT_FRAME, USE_* 등 launch override는 planner_preflight가 최종 파라미터로 다시 검사한다.

원본 출처: `../dwa_planner/config/dwa_param.yaml`, `../dwa_planner/config/robot_param.yaml`. 비교본 출처: `scout_dwa_noetic/dwa_planner/config/scout_dwa_param.yaml`, `scout_robot_param.yaml`. Reference: `config/dwa_scout_reference.yaml`. Synthetic: `config/synthetic_dwa.yaml`.

cloud z/range/self filter는 원본 알고리즘 파라미터가 아니라 외부 어댑터의 설정이다. `config/cloud_adapter_reference.yaml` 값과 비교본의 센서 좌표 필터를 실측으로 취급하지 않는다. LiDAR extrinsic, 필터, footprint, coverage, 품질·pose jump·정지 판정 한계와 driver timeout은 `hardware_unverified.yaml`에 null로 유지한다.

Guard의 외접원 veto는 원본의 축별 padding 확장을 포함하기 위해 최대 꼭짓점 반경에 √2×padding을 더한다. 이동·회전 명령을 0으로 차단하며 다른 궤적을 고르지 않는다. distant/future-path 샘플링 누락은 이 veto로 해결되지 않는다.
