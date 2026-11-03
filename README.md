본 프로젝트의 목표는 foundationpose 의 파이프 6D pose를 입력으로 하여 2개의 파이프가 정렬되도록 Xarm 제어하는 것이다.

수동 teleop으로 grasping이 끝난 상태에서 시작하며, 이후 정렬을 자율제어한다.

사용 하드웨어

- 제어 PC : slave와 master 및 teleop 관리
- 비전 PC : foundationpose 및 anygrasp 관리
- slave : Xarm
- master : Omega 7
- camera : zed2i

1. 사전작업

- [preliminary_work/README.md](preliminary_work/README.md) 를 참고하라.

2. 이후 작업 예상

- 비전 PC의 양쪽 파이프 pose 토픽과 좌표변환(calibration)을 확인한다.
- 파이프 정렬 목표와 현재 pose를 이용하여 양팔의 목표 EEF pose를 계산한다.
- teleop에서 자율제어로 넘기는 절차를 정하고, xArm 자율제어를 구현한다.
- 실제 pose feedback으로 정렬을 보정하고, tracking lost 및 통신 중단 시 정지하도록 한다.
