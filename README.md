본 프로젝트의 목표는 foundationpose 의 파이프 6D pose를 입력으로 하여 2개의 파이프가 정렬되도록 Xarm 제어하는 것이다.

수동 teleop으로 grasping이 끝난 상태에서 시작하며, 이후 정렬을 자율제어한다.

사용 하드웨어

- 제어 PC : slave와 master 및 teleop 관리
- 비전 PC : foundationpose 및 anygrasp 관리
- slave : Xarm
- master : Omega 7
- camera : zed2i

- [vision_pc](src/vision_pc/README.md): 비전 코드의 임시 참조와 외부 의존성.
- [control_pc](src/control_pc/README.md): RoHand selector와 향후 Auto Align 개발.

1. 사전작업

- [제어 PC 준비](docs/preliminary_work/README_control_pc.md), [비전 PC 준비](docs/preliminary_work/README_vision_pc.md)를 참고하라.
- 제어 PC에서 teleop으로 두 파이프를 잡고 기본 자세까지 준비한다.
- 비전 PC에서 ZED2i 영상 발행과 FoundationPose 추적을 실행한다.

2. 이후 작업 예상

- 사전작업 완료 후 비전 PC에서 ZED2i와 FoundationPose를 실행하고, 양쪽 파이프 pose 토픽과 좌표변환(calibration)을 확인한다.
- 파이프 정렬 목표와 현재 pose를 이용하여 양팔의 목표 EEF pose를 계산한다.
- teleop에서 자율제어로 넘기는 절차를 정하고, xArm 자율제어를 구현한다.
- 실제 pose feedback으로 정렬을 보정하고, tracking lost 및 통신 중단 시 정지하도록 한다.
