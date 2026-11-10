# 좌표계 검증

공식 `xarm_description` 모델과 PyKDL로 저장된 flange 자세를 표시하거나, 실제 관절 피드백을 표시한다.

| 모드 | 동작 | 실물 연결 | 이동 명령 |
|---|---|---|---|
| `static_rviz` (기본) | 선택한 자세 행렬을 정적으로 표시 | 없음 | 없음 |
| `dynamic_rviz` | 선택한 자세 사이의 경로를 화면에서 재생 | 없음 | 없음 |
| `live_feedback` | 실제 관절각으로 화면 갱신 | 필요 | 없음 |
| `live_control` | C++ 제어기 이동과 실측 시각화 | 필요 | Enter 후 전송 |

`live_control`은 `ros2 run simulation_validation live_control`로 실행한다. 팔·시작·목표는 필수이며 같은 터미널의 Enter로 이동한다. `ros2 launch ... mode:=live_control`은 통합 실행기 내부의 시각화 전용 구성이다. 단독 호출은 제어기를 실행하지 않는다.
전체 명령 모음은 [제어 PC 시각화](../../../../docs/control_pc/README_simulation.md)를 따른다.

- 공통 토르소 기준, 양팔 base·flange·TCP 축을 표시한다. 토르소·손 형상은 포함하지 않는다.
- `../../config/fixed_frames.yaml`을 직접 읽으며 장착 변환과 TCP를 중복 저장하지 않는다.
- `../../config/arm_poses.yaml`의 `poses`에서 `init1`, `init1_floor_parallel_v1`, `init1_floor_parallel_v2`의 양팔 base 기준 flange 행렬을 선택한다. 위치는 m이다.
- `config/init1.yaml`은 외부 SET1 관절각을 기록한 로컬 IK 초기 추정값이다. degree를 rad로 변환해 사용한다.
- 공식 URDF의 회전축·관절 원점·관절 제한으로 KDL 체인을 구성한다. 정적·동적 모드는 SDK나 로봇 접속 없이 계산한다.
- 카메라는 토르소 고정이다. 현재 장착 변환이 미확인이므로 표시하지 않는다. `camera_mount.parent_T_camera`에 `translation_m`과 `rpy_rad` 또는 `rotation_matrix`를 채우면 축을 표시한다.
- 시각화 토픽·TF는 `/simulation_validation/` 아래에 둔다. `live_control`에서만 `/xarm_motion/joint_states`·`/xarm_motion/status`를 구독해 실측 관절각과 상태를 전달한다.
- `live_control --step pre_align_ready`는 `/auto_align/joint_states`·`/auto_align/status`를 사용한다.

## 공식 모델 준비

아래는 공식 모델을 캐시 경로에 두고 함께 빌드하는 절차다. 저장소 루트에서 모델 다운로드를 처음 한 번 실행한다.
외부 기존 파일은 수정하지 않으며, 내려받은 모델은 Git에서 제외된 `_colcon/dependencies/`에 둔다.

```bash
cd ~/pipe_align_control_kkb
git clone --depth 1 --filter=blob:none --sparse --branch humble \
  https://github.com/xArm-Developer/xarm_ros2.git \
  main/control_pc/_colcon/dependencies/xarm_ros2
git -C main/control_pc/_colcon/dependencies/xarm_ros2 sparse-checkout set xarm_description
```

확인한 공식 모델 버전은 `ac736c92b02de7ca7e0be6f737d77d46707464d8`이다.
공식 모델 원본과 기구학을 수정하지 않고, 생성된 시각화 URDF에서 제어·Gazebo 관련 태그만 제외한다.

## 빌드

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
mkdir -p main/control_pc/_colcon
touch main/control_pc/_colcon/COLCON_IGNORE
colcon --log-base main/control_pc/_colcon/log build \
  --base-paths main/control_pc/src \
    main/control_pc/_colcon/dependencies/xarm_ros2/xarm_description \
  --packages-select xarm_description simulation_validation \
  --build-base main/control_pc/_colcon/build \
  --install-base main/control_pc/_colcon/install \
  --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF
```

## 표시

새 터미널에서 실행한다. 이 명령에는 로봇 초기화나 이동 동작이 없다.

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
export ROS_LOG_DIR=~/pipe_align_control_kkb/main/control_pc/_log/simulation_validation
ros2 launch simulation_validation init1.launch.py mode:=static_rviz pose_name:=init1
```

RViz에서 실제 `init 1` 자세와 양팔 장착 방향·관절 배치·flange·TCP 축을 비교한다. 좌표축의 X/Y/Z는 빨강/초록/파랑이다.
전체 관절 좌표계와 카메라 축은 `All link frames (optional)` 표시를 켜서 확인할 수 있다.
기존 `T_sr`와 TCP offset의 실측 정확도, 실측값과 모델의 차이는 별도 확인이 필요하다. 행렬을 IK로 표시한 초기 관절각은 SET1과 최대 약 2.1° 다를 수 있다.
표준 xArm7 기본 기구학을 사용하며 개체별 보정은 아직 적용하지 않는다.
1300 계열 도구 장착면 모델이 필요한 경우 해당 팔에 `model1300_left:=true` 또는 `model1300_right:=true`를 추가한다.

GUI 없이 관절각·TF만 확인하려면 같은 launch에 `rviz:=false`를 추가한다. 종료는 Ctrl+C다.
공통 설정 경로는 자동으로 찾으며, 다른 파일을 확인할 때는 `frames_file:=/절대경로/fixed_frames.yaml`을 지정한다.
VS Code Snap의 GTK 경로는 RViz 프로세스에서만 제외하며 시스템 환경은 변경하지 않는다.

## 화면에서 이동 재생

```bash
ros2 launch simulation_validation init1.launch.py mode:=dynamic_rviz \
  start_pose:=init1 target_pose:=init1_floor_parallel_v1 duration:=5.0
```

시작 자세를 2초 유지한 뒤 구간당 `duration`초 동안 한 번 재생하고 최종 자세를 유지한다.
`start_pose`와 `target_pose`는 필수이며 생략하면 실행 전에 오류로 종료한다.
위치·회전을 보간하고 직전 관절각을 기준으로 IK를 계산한다. 전체 경로의 IK·관절 제한·인접 관절 변화 15° 제한을 통과해야 재생한다.
IK 실패 시 launch를 종료한다. 실제 충돌이나 동역학을 검사하는 기능은 없다.
`Goal flange axes`는 최종 목표 축이며, `pose_name:=init1_floor_parallel_v1`을 정적 모드에 지정하면 목표만 표시한다.
v1은 flange 위치를 유지한 방향 보정이고, v2는 왼팔 world X·Y와 오른팔 목표 자세를 유지하며 왼팔 높이를 오른팔에 맞춘 목표다. 원하는 버전을 `target_pose`로 선택한다.
여러 자세를 재생하려면 `start_pose:=init1 target_pose:=init1 pose_sequence:=init1,init1_floor_parallel_v1,init1`을 사용한다. 목록의 양 끝은 명시한 시점·종점과 같아야 한다.
자세 파일은 `coordinates_file`, 초기 추정 관절각은 `seed_file`로 변경할 수 있다.

## 실제 관절각 표시

`live_feedback`는 각 팔의 관절각을 20 Hz로 조회해 같은 모델에 표시한다. 기존 정적·동적 시각화 창은 Ctrl+C로 종료한 뒤 실행한다.
한 launch 안에서는 선택한 관절각 발행기 하나만 실행된다.
공식 `xarm-python-sdk`가 현재 ROS Python 환경에 설치되어 있어야 한다. 정적·동적 모드에는 SDK가 필요 없다.

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
export ROS_LOG_DIR=~/pipe_align_control_kkb/main/control_pc/_log/simulation_validation
ros2 launch simulation_validation init1.launch.py mode:=live_feedback
```

- 기본 주소는 왼팔 `192.168.2.31`, 오른팔 `192.168.2.32`다. `left_ip:=주소 right_ip:=주소`로 변경할 수 있다.
- 공식 SDK의 `get_joint_states(is_radian=True, num=1)`로 각 팔 J1~J7을 읽는다. ROS의 EEF pose 토픽은 사용하지 않는다.
- `/simulation_validation/joint_states` → `robot_state_publisher` → RViz로 연결한다. 장착 변환과 TCP는 기존 `fixed_frames.yaml`을 사용한다.
- 이동, 모터 활성화, 모드·상태·TCP 설정 변경은 하지 않는다. 별도 제어기로 실물을 움직이면 화면이 따라간다.
- 팔별로 수신 여부를 터미널에 표시한다. 새 값이 1초 이상 없거나 조회에 실패하면 경고하고 마지막 자세를 유지한다. 연결 실패 시 2초 뒤 재시도한다.
- 최초 수신 전에는 해당 팔의 관절각을 발행하지 않는다. 한쪽만 수신되면 그 팔부터 표시하며, `init1`이나 영점 관절각으로 대체하지 않는다.
- 마지막 자세도 계속 표시하므로 `joint_states.header.stamp`는 화면 갱신 시각이다. 실제 수신 여부는 터미널 경고로 확인한다. 이 토픽은 시각화 전용이다.

화면 일치는 관절각뿐 아니라 장착 변환·기구 모델의 정확도에 좌우된다. `live_feedback`에서 base 장착값이나 TCP offset을 자동 보정하지 않는다.

실행 로그는 `main/control_pc/_log/simulation_validation/`에 저장한다. 위 `ROS_LOG_DIR` 설정은 launch 자체 로그에도 적용한다.

## 실측 시각화와 함께 이동

공식 C++ SDK 환경을 불러와 `xarm_motion`과 이 패키지를 빌드한다. 실행 전 teleop·init·다른 시각화를 종료하고 실제 로봇을 시작 자세에 놓는다. 공통 `control.yaml`의 `execution.standalone_enabled`를 파일에서 직접 `true`로 설정한다.

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source ~/project/xarm_sdk/install/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 run simulation_validation live_control \
  --arm both --start-pose init1 --target-pose init1_floor_parallel_v2
```

팔·시작·목표는 필수다. 모델·환경 준비 후 C++ 제어기를 실행하며 실제 시작 자세·전체 경로 검사를 통과하면 `READY`를 표시한다. 같은 터미널의 빈 줄 Enter로 이동한다. Ctrl+C와 이동 중 시각화 종료·실측 수신 중단 시에는 제어기에 중지를 요청한다. SDK 접속과 제어는 C++ 실행기 하나만 담당한다.

실측 관절각을 20 Hz로 전달하며 최초 수신 전에는 임의 자세를 넣지 않는다. 목표 flange 축과 제어 상태를 함께 표시한다. 정상 완료 후 제어기는 중지·종료하고 RViz는 마지막 수신 자세와 완료 상태를 유지한다. 이후 실물은 조회하지 않으며 Ctrl+C로 화면을 닫는다. `--no-rviz`는 GUI 없이 같은 연결을 실행한다.

공통 좌표계·자세 파일과 `control.yaml`의 모델 설정을 사용한다. 이동 시간·제한·도착 검사는 기존 제어기에 맡긴다. 통합 실행기·시각화 로그는 `_log/simulation_validation/`, 제어기 로그는 `_log/xarm_motion/`에 저장한다. 빌드와 상세 절차는 [제어 PC 시각화](../../../../docs/control_pc/README_simulation.md)를 따른다.

`auto_align`을 빌드한 뒤 `ros2 run simulation_validation live_control --arm both --step pre_align_ready`로 현재 자세에서 준비 목표까지 실행할 수 있다. 시작·목표 인자는 받지 않으며 `execution.auto_align_enabled=true`가 별도로 필요하다. 전체 SDK 경로와 팔·환경의 기본 20 mm 충돌 여유거리를 검사하고 Enter로 실행한다. 손·파이프·토르소 형상과 자동 우회는 포함하지 않는다. 제어 로그는 `_log/auto_align/`이다.

## 환경 모델

[config/environment_models.yaml](config/environment_models.yaml)의 책상·블라인드·오른쪽 벽을 모든 모드에서 기본 표시한다.
책상 가까운 모서리 윗면 중앙은 `table.position_m`·`rpy_rad`, 바닥은 `floor_z_m`으로 world 기준에서 직접 지정한다.
world +X는 전방, +Y는 왼쪽, +Z는 위쪽이다. 환경 배치는 로봇 장착값이나 저장 자세를 참조하지 않는다.
블라인드는 먼 모서리에서 3 cm 떨어진 폭 152 cm의 판이고, 오른쪽 벽은 옆 모서리에서 안쪽 면까지 10 cm 간격인 길이 76 cm의 판이다.
두 벽은 바닥부터 높이 2 m이며 책상의 수평 배치를 따라간다. 로봇 장착·목표 자세는 변경하지 않는다.
YAML에서 크기·간격·추가 이동·회전·색상·표시 여부를 변경한 뒤 재시작하면 적용된다. 재빌드는 필요 없다.
`show_environment:=false`로 전체를 숨기고 `environment_file`로 다른 파일을 선택한다. 표시용 모델이며 충돌 검사는 하지 않는다.

## 검사

빌드 환경을 불러온 상태에서 실행한다. 로봇은 사용하지 않는다.

```bash
python3 -m unittest discover \
  -s main/control_pc/src/simulation_validation/test -p 'test_*.py' -v
```
