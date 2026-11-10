# 양팔 xArm 단독 제어 — 제어 PC

`xarm_motion`으로 실제 양팔을 `init1`에서 `init1_floor_parallel_v2`로 이동한다.

## 1. 실행 준비

실제 로봇을 `init1` 자세에 놓고 teleop·init 등 다른 xArm 제어 프로그램을 종료한다.
`xarm_motion` 빌드가 완료된 상태에서 실행한다. 기존 단독 이동은 SDK 피드백 기준으로 검증했으며 실제 바닥 평행도·world 장착 정합은 별도 확인이 필요하다.

## 2. 단독 실행 허용

`main/control_pc/config/control.yaml`의 값을 직접 변경한다. YAML 변경 후 재빌드는 필요 없다.

```yaml
execution:
  standalone_enabled: true
```

`false`이면 로봇 연결 전에 경고 후 종료한다. 실행 옵션으로 변경할 수 없다.

## 3. 터미널 1 — init1에서 v2로 이동

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source ~/project/xarm_sdk/install/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 run xarm_motion xarm_motion \
  --arm both \
  --start-pose init1 \
  --target-pose init1_floor_parallel_v2
```

- `--arm`: `left`, `right`, `both` 중 선택한다. 팔·시작·목표 인자는 모두 필수다.
- 시작·목표 이름은 `main/control_pc/config/arm_poses.yaml`에서 읽는다.
- SDK FK·IK로 전체 경로를 먼저 계산한다. `READY`와 예상 이동 시간을 확인한 뒤 빈 줄 **Enter**로 이동을 시작한다.
- **Ctrl+C**는 선택한 팔 모두에 중지를 요청하고 종료한다. 도착 확인 후에도 중지하고 종료한다.
- 실제 자세가 시작 조건과 다르거나 제한값 검사를 통과하지 못하면 이동을 거부한다.

실행 로그는 `main/control_pc/_log/xarm_motion/`에 저장한다.

사용 후 `control.yaml`의 `standalone_enabled`를 `false`로 되돌린다.

## 4. RViz와 함께 실행

`simulation_validation`과 `xarm_motion`을 빌드한 뒤 위 단독 실행 대신 사용한다. 다른 시각화·제어 실행은 먼저 종료한다.

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source ~/project/xarm_sdk/install/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 run simulation_validation live_control \
  --arm both \
  --start-pose init1 \
  --target-pose init1_floor_parallel_v2
```

실측 자세·목표 축·책상·벽을 표시한다. 같은 터미널에서 `READY` 후 빈 줄 Enter로 이동하고 Ctrl+C로 중지한다. 완료 후 제어기는 중지·종료하며 RViz는 마지막 수신 자세와 `COMPLETED`를 유지한다. 이후 실물 변화는 조회하지 않는다. Ctrl+C로 화면을 닫는다. 이동 중 시각화 종료·실측 수신 중단 시 제어기에 중지를 요청한다.

빌드와 상세 동작은 [시각화 실행 문서](README_simulation.md)의 `live_control` 항목을 따른다.

## 5. 현재 자세에서 pre_align_ready로 이동

`auto_align`을 [패키지 안내](../../main/control_pc/src/auto_align/README.md)에 따라 빌드한다. teleop·init·다른 팔 제어기를 종료하고 손의 grasp 상태는 유지한다.
`control.yaml`의 `execution.auto_align_enabled`를 파일에서 직접 `true`로 바꾼다. 기본값은 `false`이며 기존 `standalone_enabled`와 별도다.

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source ~/project/xarm_sdk/install/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 run simulation_validation live_control --arm both --step pre_align_ready
```

시작 자세 이름은 지정하지 않는다. 현재 방향·world X·Y를 유지하며 각 팔의 준비 목표 높이로 이동하고, 방향을 유지한 채 목표 X·Y로 이동한 뒤 목표 방향을 맞춘다. 양팔은 구간별 시간을 맞춘다.
전체 SDK 경로와 팔·환경의 20 mm 여유거리 검사를 통과하면 `READY`가 나온다. 같은 터미널의 빈 줄 Enter로 한 번 실행한다. 충돌 경로는 거부하며 자동 우회하지 않는다.
손·파이프·토르소 형상은 검사하지 않는다. 20 mm는 모델 기준이며 실물 배치·추종 오차를 포함한 안전 보증은 아니다.
오류·Ctrl+C·완료 시 양팔에 중지를 요청한다. 완료 후 RViz는 마지막 수신 자세를 유지한다. FoundationPose는 자동 실행하지 않는다.

RViz 없이 제어기만 실행하려면 `ros2 run auto_align auto_align --step pre_align_ready`를 사용한다.
사용 후 `execution.auto_align_enabled`를 `false`로 되돌린다. 실물 검증은 사용자가 직접 실행·Enter 입력으로 수행한다.
