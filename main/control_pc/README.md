# 제어 PC

- `src/rohand_selector/`: 직접 개발한 손 선택 wrapper다. 패키지 내부 코드와 설정을 유지한다.
- `src/auto_align/`: 현재 실측 자세에서 준비 목표로 이동하는 `pre_align_ready` 단계다. 수직·수평·방향 구간을 SDK로 계산하고 모델 충돌을 검사한다. FoundationPose 미세 정렬은 아직 연결하지 않았다. [실행 방법](src/auto_align/README.md)을 따른다.
- `src/xarm_motion/`: C++ 양팔 이동 패키지다. 파일에서 실행을 허용하고 Enter 실행·Ctrl+C 중지를 사용한다. 단독 실행과 `live_control` 통합 실행을 지원한다. [실행 방법](src/xarm_motion/README.md)을 따른다.
- `src/simulation_validation/`: 공식 xArm 모델의 정적 자세·동적 경로·실측 관절각 표시와 `live_control` 통합 실행을 담당한다. 환경 설정은 패키지의 `config/environment_models.yaml`에 둔다. [실행 방법](src/simulation_validation/README.md)을 따른다.
- `config/`: 고정 좌표계 `fixed_frames.yaml`, 양팔 자세 `arm_poses.yaml`, 이동 제한·실행 허용·충돌 여유거리를 정하는 `control.yaml`을 둔다. 좌표계·자세 설정은 시각화에서도 읽는다.
- `src_external/`: 외부 teleop·Omega7·xArm·RoHand의 경로를 `external_path.yaml`, 실행 설정을 `external_config.yaml`에 기록한다. 외부 소스는 기존 위치에서 참조하며, 두 YAML은 참조용이다.
- `_log/`: `xarm_motion/`, `auto_align/`, `rohand_selector/`, `simulation_validation/`의 실행 로그와 calibration 기록을 모은다. Git 추적에서 제외한다.
- `_colcon/`: 빌드·설치·공식 모델 캐시와 colcon 빌드·테스트 로그를 모은다. 패키지 탐색과 Git 추적에서 제외한다.

실행은 [제어 PC 준비](../../docs/preliminary_work/README_control_pc.md)를 따른다. 저장소 루트에서 selector를 빌드하되 산출물은 `_colcon/`에 저장하며, 외부 프로그램은 기존 위치에서 사용한다.
