# 출처 및 라이선스

- [AMSL dwa_planner](https://github.com/amslabtech/dwa_planner)의 `6387a9660c2064fa0545ac81d55f4bbe7962651f`를 바탕으로 Scout 주행에 맞게 수정했습니다. [BSD 3-Clause 라이선스](../local_ws/src/dwa_planner/LICENSE)를 포함합니다.
- [지도 작성 런치](../launch/mapping.launch)는 [HDL Graph SLAM](https://github.com/koide3/hdl_graph_slam)의 `95b8dce41c10667dcf0e3e9801dc22754b3bd525`를 바탕으로 장비 설정을 반영한 사본입니다. [원본 라이선스](../launch/HDL_GRAPH_SLAM_LICENSE)를 포함합니다.
- `scout_dwa`의 라이선스 선언은 BSD-3-Clause이며, `use_yolo`의 라이선스 선언은 MIT입니다. 각 패키지의 `package.xml`을 참고하세요.
- 기타 외부 ROS 패키지의 출처와 커밋은 [dependencies.repos](../dependencies.repos)에 있습니다. 각 패키지는 해당 저장소의 라이선스를 따릅니다.
- PCD 지도와 ONNX 모델은 프로젝트에서 사용한 실행 자료입니다. 모델의 학습 데이터·학습 코드는 이 저장소에서 확인되지 않으며, 모델과 지도의 별도 배포 조건은 확인이 필요합니다.
