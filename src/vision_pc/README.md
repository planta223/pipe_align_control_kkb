# 비전 PC

- `temp_adaptor/`: 직접 개발한 FoundationPose 기반 다중 파이프 추적·후처리·ROS pose 발행 코드의 임시 참조다. 원본은 `~/Pipe_Align_kkb/MULTI_OBJECT_TRACKING`에 있으며, 추후 이 저장소로 이전한다.
- `external_reference/`: 공식 FoundationPose 기반 환경과 Docker, ZED2i ROS 카메라 publisher, AnyGrasp의 참조다. AnyGrasp 경로는 아직 확인하지 않았으며 현재 수동 grasp 후 정렬의 필수 경로가 아니다.
- 각 `*_path.yaml`은 위치, `*_config.yaml`은 실행 조건을 기록한다. 모두 현재 미적용이며 wrapper는 없다. 기존 명령과 원본 프로그램의 동작은 유지한다.
- `config/`: 향후 이 저장소의 비전 프로그램 설정을 둘 위치다.

실행은 기존 [비전 PC 준비](../../docs/preliminary_work/README_vision_pc.md)를 따른다. 현재 ROS 출력은 left Pipe만 지원한다.
