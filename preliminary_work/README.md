# Auto Align 사전작업

제어 PC에서 매니퓰레이터 초기화와 teleop 설정을 하고, 두 Pipe를 잡은 기본 자세까지 준비한다.
FoundationPose는 이 사전작업이 끝난 후 비전 PC에서 별도로 실행한다.
Control PC에서 터미널 1~5를 사용한다. 각 실행 프로그램은 해당 터미널에서 켜 둔다.

## 장치 준비

Omega7, 양팔 xArm, RoHand 전원과 연결을 확인한다.

## 1. 터미널 1 — Omega7 초기화 및 teleop

초기화 중 장치가 움직이므로 손을 뗀다.

```bash
cd ~/Omega7
./autoinit_both
```

초기화가 정상 완료되어 prompt가 돌아오면 같은 터미널에서 teleop을 실행한다.
Omega master와 양팔 xArm 제어가 함께 켜진다. xArm 제어 launch를 따로 중복 실행하지 않는다.

```bash
cd ~/Omega7
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
./run_teleop.sh
```

## 2. 터미널 2 — RoHand /leftarm

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

## 3. 터미널 3 — RoHand /rightarm

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

## 4. 터미널 4 — 손 개폐 발판

양손

```bash
cd ~/project/robothand_ws/rohand_ctrl_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 launch rohand_foot_pedal_controller foot_pedal_hand_controller.launch.py
```

오른손 (사용자 실물 확인)

```bash
ros2 launch rohand_foot_pedal_controller foot_pedal_hand_controller.launch.py \
  default_task_num:=2
```

반대손 (실물 확인 필요)

```bash
ros2 launch rohand_foot_pedal_controller foot_pedal_hand_controller.launch.py \
  default_task_num:=1
```

- 기본 task 0은 **양손 동시 제어**다. task 입력이 있으면 대상 손이 바뀌므로 로그를 확인한다.
- 발판을 누를 때마다 닫기/열기가 바뀐다. 프로그램 시작 후 첫 입력은 닫기다.
- 프로그램을 켜기만 해서는 실제 손이 열리지 않는다.

## 수동 grasping

1. Omega grip을 먼저 열고, 쥔 상태에서 팔을 이동한다.
2. Pipe를 잡을 위치에서 Omega grip을 놓아 팔 이동을 멈춘다.
3. 발판으로 손을 닫아 두 Pipe를 잡는다.
4. 두 Pipe를 잡은 상태에서 teleop으로 기본 자세로 이동한 뒤 Omega grip을 놓는다.

Omega grip은 팔 이동용 clutch이고, 실제 손 개폐는 발판으로 한다.

## 5. 터미널 5 — 준비 확인

```bash
source ~/Omega7/setup_ros2.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 topic list -t
ros2 topic echo --once /left/robot/current/base_eef_pose
ros2 topic echo --once /right/robot/current/base_eef_pose
```

두 Pipe의 grasp와 기본 자세가 유지되고, 양팔 EEF feedback이 갱신되면 제어 PC 준비 완료다.

자율제어 전환 절차는 아직 미정이다. teleop과 별도 자율제어기를 같은 팔에 동시에 실행하지 않는다.

## 비전 PC — FoundationPose 준비

제어 PC에서 두 Pipe를 잡고 기본 자세를 마친 후 진행한다.
소스 확인과 실행 절차 정리는 **비전 PC의 Codex로 진행한다.**

1. 기존 FoundationPose 프로젝트와 실행 환경을 확인한다.
2. ZED2i 입력을 준비하고 두 Pipe의 pose tracking을 실행한다.
3. 양쪽 pose 토픽, 좌표계, 단위, timestamp를 확인한다.
4. 제어 PC에서 두 Pipe의 pose가 수신되는지 확인한다.

터미널별 실행 명령은 비전 PC에서 확인 후 여기에 추가한다.
