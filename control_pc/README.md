# 제어 PC

- `rohand_selector/`: 직접 개발한 손 선택 wrapper다. 패키지 내부 코드와 설정을 유지한다.
- `auto_align/`: 향후 직접 개발할 파이프 정렬 제어 코드의 위치다. 아직 구현하지 않았다.
- `config/`: 향후 제어 PC 설정을 둘 위치다.
- `external_reference/`: 외부 teleop·Omega7·xArm·RoHand 참조를 둘 위치다. 실제 제어 PC에서 확인하고 연결할 예정이며, 새 path/config YAML은 두지 않는다.

실행은 [제어 PC 준비](../docs/preliminary_work/README_control_pc.md)를 따른다. 저장소 루트에서 selector를 빌드하며, 외부 프로그램은 기존 위치에서 사용한다.
