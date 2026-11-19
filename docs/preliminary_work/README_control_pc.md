# Auto Align 사전작업

제어 PC에서 매니퓰레이터 초기화와 teleop 설정을 하고, 두 Pipe를 잡은 기본 자세까지 준비한다.
FoundationPose는 이 사전작업이 끝난 후 비전 PC에서 별도로 실행한다.
Control PC에서 터미널 1~7을 사용한다. 초기화 프로그램은 완료 후 종료되며, 나머지 실행 프로그램은 해당 터미널에서 켜 둔다.

## 장치 준비

Omega7, 양팔 xArm, RoHand 전원과 연결을 확인한다.

## 1. 터미널 1 — Omega7 초기화

초기화 중 장치가 움직이므로 손을 뗀다.

```bash
cd ~/Omega7
./autoinit_both
```

## 2. 터미널 2 — xArm 초기 자세 이동

teleop 시작 전, Pipe를 잡지 않은 상태에서 실행한다. 양팔이 저장된 초기 자세 set 1로 동시에 이동한다.

```
0: 양팔 뒤쪽으로
1: 양팔 뒤쪽으로 (0과 동일)
2: 양팔 앞쪽으로
3 이상은 사용 금지
```

```bash
source ~/project/xarm_sdk/install/setup.bash
cd ~/project/xarm_sdk/install/xarm_sdk/lib/xarm_sdk
./init 1
```

양팔의 `Successfully moved to initial pose.` 출력을 확인한 뒤 다음 단계로 간다.

## 3. 터미널 3 — teleop

Omega7 초기화와 양팔 초기 자세 이동이 완료된 후 실행한다.
Omega master와 양팔 xArm 제어가 함께 켜진다. xArm 제어 launch를 따로 중복 실행하지 않는다.

```bash
cd ~/Omega7
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
./run_teleop.sh
```

## 4. 터미널 4 — RoHand /leftarm

손 드라이버는 시작 시 힘 센서를 영점화하므로 Pipe를 잡기 전에 실행한다.
`/leftarm`, `/rightarm`은 설정상 이름이며, USB0/1과 실제 손의 좌우 대응은 확인이 필요하다.

```bash
cd ~/project/robothand_ws/rohand_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source rohand/bin/activate
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 run rohand rohand_modbus_ap001 --ros-args \
  -p port_name:=/dev/ttyUSB0 -p baudrate:=115200 \
  -p 'hand_ids:=[2]' -r __ns:=/leftarm
```

## 5. 터미널 5 — RoHand /rightarm

```bash
cd ~/project/robothand_ws/rohand_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source rohand/bin/activate
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 run rohand rohand_modbus_ap001 --ros-args \
  -p port_name:=/dev/ttyUSB1 -p baudrate:=115200 \
  -p 'hand_ids:=[2]' -r __ns:=/rightarm
```

## 6. 터미널 6 — 손 개폐 발판

기존 발판 프로그램이 켜져 있으면 그 프로그램만 종료한다. 손 드라이버와 teleop은 유지한다.

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source ~/project/robothand_ws/rohand_ctrl_ws/install/local_setup.bash
source main/control_pc/_colcon/install/local_setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 run rohand_selector rohand_selector
```

- 이 창에서 `l`/`ㅣ`: 왼손, `r`/`ㄱ`: 오른손, `Space`: 양손을 선택한다. 키 목록과 task 번호는 `selector.yaml`에서 관리하며, 등록하지 않은 키는 무시한다.
- 선택된 손 이름은 빨간색으로 표시한다. 한글은 입력기에서 확정된 문자부터 인식한다.
- 키는 선택만 바꾼다. `선택:` 표시를 확인한 뒤 발판으로 닫기/열기를 한다. 선택 변경 시 손별 개폐 상태는 유지된다.
- 시작 선택은 양손이다. 프로그램 시작 시 자동 개폐하지 않으며, 첫 발판 입력은 닫기다. 재시작하면 내부 개폐 상태는 초기화된다.
- Ctrl+C는 selector와 자신이 시작한 발판 프로그램만 종료한다. 열기 명령은 보내지 않는다.
- 키/task 대응은 [selector.yaml](../../main/control_pc/src/rohand_selector/config/selector.yaml), 기존 프로그램의 전체 출력은 `main/control_pc/_log/rohand_selector/`에 저장한다.

## 수동 grasping

1. Omega grip을 먼저 열고, 쥔 상태에서 팔을 이동한다.
2. Pipe를 잡을 위치에서 Omega grip을 놓아 팔 이동을 멈춘다.
3. 발판으로 손을 닫아 두 Pipe를 잡는다.
4. 두 Pipe를 잡은 상태에서 teleop으로 기본 자세로 이동한 뒤 Omega grip을 놓는다.

Omega grip은 팔 이동용 clutch이고, 실제 손 개폐는 발판으로 한다.

## 7. 터미널 7 — 준비 확인

```bash
source ~/Omega7/setup_ros2.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 topic list -t
ros2 topic echo --once /left/robot/current/base_eef_pose
ros2 topic echo --once /right/robot/current/base_eef_pose
```

두 Pipe의 grasp와 기본 자세가 유지되고, 양팔 EEF feedback이 갱신되면 제어 PC 준비 완료다.

준비 이동은 손의 grasp 상태를 유지한 채 teleop·init·다른 팔 제어기를 종료하고 `auto_align`의 `pre_align_ready` 단계로 수동 전환한다. [실행 방법](../control_pc/README_only_control.md)의 준비 이동 항목을 따른다. teleop과 자율제어기를 같은 팔에 동시에 실행하지 않는다. 준비 이동 후 FoundationPose와 미세 정렬의 자동 연결은 아직 구현하지 않았다.

## 비전 PC — FoundationPose 준비

제어 PC에서 두 Pipe를 잡고 기본 자세를 마친 후 진행한다.
소스 확인과 실행 절차 정리는 README_vision_pc.md 를 참고하라.

## 8. 터미널 8 — 비전 pose 수신 확인

비전 PC에서 [카메라와 FoundationPose](README_vision_pc.md)를 실행하고 두 Pipe tracking을 유지한다.
아래 명령은 제어 PC의 새 터미널에서 실행한다. 별도 빌드나 구독 프로그램은 필요하지 않다.

### 양쪽 publisher 확인

```bash
ros2 topic info -v /vision/left_pipe/pose
ros2 topic info -v /vision/right_pipe/pose
```

두 토픽 모두 `Publisher count`가 1 이상이고, 타입이 `geometry_msgs/msg/PoseStamped`인지 확인한다.
문서상 publisher QoS는 BEST_EFFORT / VOLATILE / depth 1이다.

### 실제 메시지 수신 확인

```bash
timeout 10s ros2 topic echo --once /vision/left_pipe/pose \
  --qos-reliability best_effort
timeout 10s ros2 topic echo --once /vision/right_pipe/pose \
  --qos-reliability best_effort
```

- 제어 PC에서 양쪽 모두 `header`와 `pose`가 출력되어야 실제 수신 성공이다. 토픽 발견만으로 판단하지 않는다.
- `frame_id`는 `camera_left_optical_frame`이며, 위치는 m, 회전은 quaternion `x,y,z,w`다. 로봇 EEF가 아닌 Pipe pose다.
- 출력 없이 10초 뒤 종료되면 해당 토픽의 수신은 아직 확인되지 않은 상태다.

### 지속 갱신 확인

아래 명령을 하나씩 실행한다. 각 토픽의 `header.stamp`가 계속 갱신되는지 확인하고 Ctrl+C로 종료한다.

```bash
ros2 topic echo /vision/left_pipe/pose --qos-reliability best_effort
```

```bash
ros2 topic echo /vision/right_pipe/pose --qos-reliability best_effort
```

별도 `tracking_status` 토픽은 없다. 실제 손과 left/right Pipe 대응도 비전 PC의 초기 mask와 `--left-pipe-object-id` 설정으로 확인한다.
이 단계는 PC 간 수신 확인이며, 자율 정렬 제어 연결은 포함하지 않는다.

## 9. 터미널 9 — 실제 양팔 자세 시각화 / 준비 이동

### 실제 양팔 자세 시각화 — 선택 실행

공식 모델과 `simulation_validation`을 [패키지 안내](../../main/control_pc/src/simulation_validation/README.md#빌드)에 따라 빌드한 뒤 새 터미널에서 실행한다.
기존 정적 시각화가 실행 중이면 해당 창만 Ctrl+C로 종료한다.

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
export ROS_LOG_DIR=~/pipe_align_control_kkb/main/control_pc/_log/simulation_validation
ros2 launch simulation_validation init1.launch.py mode:=live_feedback
```

왼팔 `192.168.2.31`, 오른팔 `192.168.2.32`에서 공식 SDK로 관절각만 읽어 RViz에 반영한다.
로봇 이동·모터 활성화·모드 변경 명령은 보내지 않는다. 실물을 별도 제어기로 움직이면 화면이 따라간다.
수신 중단 시 경고하며 마지막 자세를 유지하므로 터미널의 양팔 수신 상태를 함께 확인한다.
주소는 `left_ip:=주소 right_ip:=주소`로 변경할 수 있다. 기본 정적 표시는 `mode:=live_feedback`를 빼고 실행한다.

정적·동적 시각화와 자세 선택 명령은 [시각화 안내](../control_pc/README_simulation.md)를 따른다.
저장 자세 사이의 실물 이동을 시각화와 함께 실행하려면 같은 문서의 `live_control` 항목을 따른다. `live_feedback`과 동시에 실행하지 않는다.

### RViz 없이 pre-align 제어만 실행

손의 grasp 상태를 유지한 채 teleop·init·다른 팔 제어기를 종료하고 실행한다.
`main/control_pc/config/control.yaml`의 `execution.auto_align_enabled`가 `true`여야 한다.

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source ~/project/xarm_sdk/install/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 run auto_align auto_align --step pre_align_ready --auto-start
```

위 명령은 `--auto-start`가 있으므로 준비·검사 후 자동으로 양팔 준비 이동을 시작한다.
수동으로 확인한 뒤 실행하려면 이 옵션을 빼고 `READY`에서 Enter를 누른다.
RViz는 시작하지 않으며, 완료하면 제어기가 종료된다. Ctrl+C로 중지할 수 있다.

준비와 검사를 마친 뒤 Enter 없이 시작하려면 다음을 사용한다. 실행 허용·충돌 검사·Ctrl+C 중지는 유지된다.

```bash
ros2 run auto_align auto_align --step pre_align_ready --auto-start
```

RViz를 함께 실행할 때도 같은 옵션을 사용할 수 있다.

```bash
ros2 run simulation_validation live_control --arm both --step pre_align_ready --auto-start
```

## 10. 터미널 10 — vision-align 측정 / 정렬

준비 자세까지 이동한 뒤 비전 PC의 FoundationPose를 실행하고, 터미널 8에서 좌우 pose 수신을 확인한다.
이미 준비 자세이면 `pre_align_ready`를 반복 실행하지 않는다. 손의 grasp를 유지하며 teleop·init·다른 팔 제어기는 종료한다.
신규 학습은 회전·병진 각각 팔당 3회, 양팔 6회이며 복귀 이동을 생략한다. 둘 다 새로 학습하면 총 12회다.
보정 이동은 별도이며, 유효한 저장값이 있으면 실제 보정 응답을 검증하면서 재사용한다.

### 측정 전용 — 로봇 이동 없음

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source ~/project/xarm_sdk/install/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 run auto_align auto_align --step vision_measure
```

로봇 SDK에 연결하지 않고 좌우 끝단·축 벡터·부호 있는 각도 성분을 기록한다. 측정 확인 후 Ctrl+C로 종료한다.

### RViz 없이 vision-align 실행

같은 환경에서 실행한다. `main/control_pc/config/control.yaml`의 `execution.auto_align_enabled`와
`execution.vision_align_enabled`가 모두 `true`여야 한다.

```bash
ros2 run auto_align auto_align --step vision_align
```

기본은 `READY` 후 Enter로 시작한다. 준비·검사를 마치면 바로 시작하려면 다음을 사용한다.

```bash
ros2 run auto_align auto_align --step vision_align --auto-start
```

### RViz와 함께 실행

위 직접 실행 명령 대신 다음을 사용한다. 두 제어기를 동시에 실행하지 않는다.

```bash
ros2 run simulation_validation live_control --arm both --step vision_align
```

이 명령에도 `--auto-start`를 추가할 수 있다. Ctrl+C로 정지·종료하며, 로그는
`main/control_pc/_log/auto_align/`에 저장한다. 카메라 좌표계 진단은 `/auto_align/vision_diagnostics`에서 확인한다.

### 저장값을 사용하지 않고 새로 학습

카메라 이동·캘리브레이션/추적 모델 변경·파이프를 다시 잡은 뒤에는 `vision_align.cache.context_id`를 바꾸거나
`--relearn`으로 실행한다. 이미 정렬된 팔·단계는 학습을 생략한다.

```bash
ros2 run auto_align auto_align --step vision_align --relearn
# 자동 시작을 함께 사용:
ros2 run auto_align auto_align --step vision_align --relearn --auto-start
# RViz를 함께 사용할 때는 위 명령 대신:
ros2 run simulation_validation live_control --arm both --step vision_align --relearn
```

저장 위치는 `main/control_pc/_state/vision_align/jacobians.yaml`이다. 설정은 `control.yaml`의
`vision_align.cache`에 있으며, `enabled: false`이면 읽기·저장을 모두 끈다.
기본은 24시간, 저장 위치에서 30 mm·방향에서 0.15 rad 이내의 후보만 검증한다.
검증 실패 시 해당 팔·단계만 재학습하며, 추적·충돌·이동 범위 이상은 중지한다.
step7 성공 후에만 저장하므로 실패·중단한 실행으로 기존 정상값을 덮어쓰지 않는다.
`CACHE_VALIDATE:`와 `VISION_MOVES:`로 재사용·학습 횟수를 확인한다. 검증은 보정 이동에 포함되므로 중복 합산하지 않는다.
자세한 판정 기준은 [auto_align 안내](../../main/control_pc/src/auto_align/README.md#학습값-저장검증재사용)를 따른다.
