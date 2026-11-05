# 제어 PC

- `src/rohand_selector/`: 직접 개발한 손 선택 wrapper다. 패키지 내부 코드와 설정을 유지한다.
- `src/auto_align/`: 향후 직접 개발할 파이프 정렬 제어 코드의 위치다. 아직 구현하지 않았다.
- `config/`: 향후 제어 PC 설정을 둘 위치다.
- `external_reference/`: 외부 teleop·Omega7·xArm·RoHand의 경로와 실행 설정을 YAML로 기록한다. 참조용이며 현재 실행에는 적용하지 않는다.
- `_colcon/`: 제어 PC의 빌드·설치·로그 산출물을 모은다. 패키지 탐색과 Git 추적에서 제외한다.

실행은 [제어 PC 준비](../../docs/preliminary_work/README_control_pc.md)를 따른다. 저장소 루트에서 selector를 빌드하되 산출물은 `_colcon/`에 저장하며, 외부 프로그램은 기존 위치에서 사용한다.
