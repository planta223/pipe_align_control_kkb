# Auto Align 사전작업

수동 teleop으로 두 Pipe를 잡고, FoundationPose tracking을 준비한다.
각 실행 프로그램은 별도 터미널에서 켜 둔다.

## 1. 장치 준비

Omega7, 양팔 xArm, RoHand 전원과 연결을 확인한다.

## 2. 터미널 1 — Omega7 초기화

초기화 중 장치가 움직이므로 손을 뗀다. 완료 후 prompt가 돌아오면 다음 단계로 간다.

```bash
cd ~/Omega7
./autoinit_both
```

## 3. 터미널 2 — 왼손 RoHand

손 드라이버는 시작 시 힘 센서를 영점화하므로 Pipe를 잡기 전에 실행한다.
USB0/1이 실제 왼손/오른손에 연결됐는지 확인한다.

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

## 4. 터미널 3 — 오른손 RoHand

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

## 5. 터미널 1 — teleop

Omega master와 양팔 xArm 제어가 함께 켜진다. xArm 제어 launch를 따로 중복 실행하지 않는다.

```bash
cd ~/Omega7
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
./run_teleop.sh
```

## 6. 터미널 4 — 손 개폐 발판

```bash
cd ~/project/robothand_ws/rohand_ctrl_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 launch rohand_foot_pedal_controller foot_pedal_hand_controller.launch.py
```

- 기본 task 0은 **양손 동시 제어**다. task 입력이 있으면 대상 손이 바뀌므로 로그를 확인한다.
- 발판을 누를 때마다 닫기/열기가 바뀐다. 프로그램 시작 후 첫 입력은 닫기다.
- 프로그램을 켜기만 해서는 실제 손이 열리지 않는다.

## 7. 수동 grasping

1. Omega grip을 먼저 열고, 쥔 상태에서 팔을 이동한다.
2. Pipe를 잡을 위치에서 Omega grip을 놓아 팔 이동을 멈춘다.
3. 발판으로 손을 닫아 두 Pipe를 잡는다.
4. 두 Pipe가 유지되는지 확인하고 Omega grip을 놓는다.

Omega grip은 팔 이동용 clutch이고, 실제 손 개폐는 발판으로 한다.

## 8. Vision PC — ZED2i / FoundationPose

기존 환경에서 ZED2i와 Dual Pipe FoundationPose tracking을 실행한다.
**원격 실행 명령은 미확인으로, 현장에서 확인 후 추가한다.**

## 9. 터미널 5 — 준비 확인

```bash
source ~/Omega7/setup_ros2.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 topic list -t
ros2 topic echo --once /left/robot/current/base_eef_pose
ros2 topic echo --once /right/robot/current/base_eef_pose
ros2 topic echo --once /vision/left_pipe/pose
ros2 topic echo --once /vision/left_pipe/tracking_status
```

Vision topic은 과거 확인된 왼쪽 이름이다. 오른쪽은 실제 topic 이름을 확인한다.
두 Pipe grasp 유지, 양팔 EEF 갱신, 양쪽 Pipe tracking을 확인하면 사전작업 완료다.

자율제어 전환 절차는 아직 미정이다. teleop과 별도 자율제어기를 같은 팔에 동시에 실행하지 않는다.
