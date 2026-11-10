# 비전 PC

- `src/temp_adaptor/`: 직접 개발한 FoundationPose 기반 다중 파이프 추적·후처리·ROS pose 발행 코드의 임시 참조다. 원본은 `~/Pipe_Align_kkb/MULTI_OBJECT_TRACKING`에 있으며, 추후 이 저장소로 이전한다.
- `src_external/`: 공식 FoundationPose 기반 환경과 Docker, ZED2i ROS 카메라 publisher, AnyGrasp의 경로를 `external_path.yaml`, 실행 조건을 `external_config.yaml`에 기록한다. 외부 소스는 기존 위치에서 참조한다. AnyGrasp 경로는 아직 확인하지 않았으며 현재 수동 grasp 후 정렬의 필수 경로가 아니다.
- 각 `*_path.yaml`은 위치, `*_config.yaml`은 실행 조건을 기록한다. 모두 현재 미적용이며 wrapper는 없다. 기존 명령과 원본 프로그램의 동작은 유지한다.
- `config/`: 향후 이 저장소의 비전 프로그램 설정을 둘 위치다.
- `_log/`: 실행 로그를 둘 빈 폴더다. 로그는 Git 추적에서 제외한다.
- `_colcon/`: 빌드·설치·colcon 로그를 둘 빈 폴더다. 패키지 탐색과 산출물의 Git 추적에서 제외한다.

실행은 기존 [비전 PC 준비](../../docs/preliminary_work/README_vision_pc.md)를 따른다. 좌우 Pipe pose를 발행하며, 별도 tracking status는 발행하지 않는다. 제어 PC의 실제 수신은 아직 확인 전이다.
