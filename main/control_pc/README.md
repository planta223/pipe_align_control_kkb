# 제어 PC

- `src/rohand_selector/`: 직접 개발한 손 선택 wrapper다. 패키지 내부 코드와 설정을 유지한다.
- `src/auto_align/`: 향후 직접 개발할 파이프 정렬 제어 코드의 위치다. 아직 구현하지 않았다.
- `src/xarm_motion/`: C++ 양팔 이동 패키지다. 단독 실행은 기본 차단하며 Enter 실행·Ctrl+C 중지를 사용한다. [실행 방법](src/xarm_motion/README.md)을 따른다. 시각화 패키지와는 아직 연결하지 않았다.
- `src/simulation_validation/`: 공식 xArm 모델의 정적 자세·동적 경로 표시와 실제 관절 피드백 시각화다. 환경 설정은 패키지의 `config/environment_models.yaml`에 둔다. `live_control`은 미구현 예약 옵션이다. [실행 방법](src/simulation_validation/README.md)을 따른다.
- `config/`: 고정 좌표계 `fixed_frames.yaml`, 양팔 자세 `arm_poses.yaml`, 이동 제한·단독 실행 허용을 정하는 `control.yaml`을 둔다. 좌표계·자세 설정은 시각화에서도 읽는다.
- `src_external/`: 외부 teleop·Omega7·xArm·RoHand의 경로를 `external_path.yaml`, 실행 설정을 `external_config.yaml`에 기록한다. 외부 소스는 기존 위치에서 참조하며, 두 YAML은 참조용이다.
- `_log/`: `xarm_motion/`, `rohand_selector/`, `simulation_validation/`의 실행 로그를 모은다. Git 추적에서 제외한다.
- `_colcon/`: 빌드·설치·공식 모델 캐시와 colcon 빌드·테스트 로그를 모은다. 패키지 탐색과 Git 추적에서 제외한다.

실행은 [제어 PC 준비](../../docs/preliminary_work/README_control_pc.md)를 따른다. 저장소 루트에서 selector를 빌드하되 산출물은 `_colcon/`에 저장하며, 외부 프로그램은 기존 위치에서 사용한다.
