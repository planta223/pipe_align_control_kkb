# 좌표계 검증

공식 `xarm_description` 모델과 PyKDL로 저장된 flange 자세를 표시하거나, 실제 관절 피드백을 표시한다.

| 모드 | 동작 | 실물 연결 | 이동 명령 |
|---|---|---|---|
| `static_rviz` (기본) | 선택한 자세 행렬을 정적으로 표시 | 없음 | 없음 |
| `dynamic_rviz` | 선택한 자세 사이의 경로를 화면에서 재생 | 없음 | 없음 |
| `live_feedback` | 실제 관절각으로 화면 갱신 | 필요 | 없음 |
| `live_control` | 실제 피드백과 제어 명령 전송 예정 | 미구현 | 미구현 |

`live_control`은 선택 시 안내 오류 후 종료하며 접속·명령 전송을 하지 않는다.
전체 명령 모음은 [제어 PC 시각화](../../../../docs/control_pc/README_simulation.md)를 따른다.

- 공통 토르소 기준, 양팔 base·flange·TCP 축을 표시한다. 토르소·손 형상은 포함하지 않는다.
- `../../config/coordinate_frames.yaml`을 직접 읽으며 장착 변환과 TCP를 중복 저장하지 않는다.
- `../../config/control_coordinate.yaml`의 `poses`에서 `init1`, `init1_floor_parallel` 등의 양팔 base 기준 flange 행렬을 선택한다. 위치는 m이다.
- `config/init1.yaml`은 외부 SET1 관절각을 기록한 로컬 IK 초기 추정값이다. degree를 rad로 변환해 사용한다.
- 공식 URDF의 회전축·관절 원점·관절 제한으로 KDL 체인을 구성한다. 정적·동적 모드는 SDK나 로봇 접속 없이 계산한다.
- 카메라는 토르소 고정이다. 현재 장착 변환이 미확인이므로 표시하지 않는다. `camera_mount.parent_T_camera`에 `translation_m`과 `rpy_rad` 또는 `rotation_matrix`를 채우면 축을 표시한다.
- 모든 발행·구독 토픽은 `/simulation_validation/` 아래에 둔다. 기존 제어 토픽과 TF에 연결하지 않는다.

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
ros2 launch simulation_validation init1.launch.py mode:=static_rviz pose_name:=init1
```

RViz에서 실제 `init 1` 자세와 양팔 장착 방향·관절 배치·flange·TCP 축을 비교한다. 좌표축의 X/Y/Z는 빨강/초록/파랑이다.
전체 관절 좌표계와 카메라 축은 `All link frames (optional)` 표시를 켜서 확인할 수 있다.
기존 `T_sr`와 TCP offset의 실측 정확도, 실측값과 모델의 차이는 별도 확인이 필요하다. 행렬을 IK로 표시한 초기 관절각은 SET1과 최대 약 2.1° 다를 수 있다.
표준 xArm7 기본 기구학을 사용하며 개체별 보정은 아직 적용하지 않는다.
1300 계열 도구 장착면 모델이 필요한 경우 해당 팔에 `model1300_left:=true` 또는 `model1300_right:=true`를 추가한다.

GUI 없이 관절각·TF만 확인하려면 같은 launch에 `rviz:=false`를 추가한다. 종료는 Ctrl+C다.
공통 설정 경로는 자동으로 찾으며, 다른 파일을 확인할 때는 `frames_file:=/절대경로/coordinate_frames.yaml`을 지정한다.
VS Code Snap의 GTK 경로는 RViz 프로세스에서만 제외하며 시스템 환경은 변경하지 않는다.

## 화면에서 이동 재생

```bash
ros2 launch simulation_validation init1.launch.py mode:=dynamic_rviz \
  start_pose:=init1 target_pose:=init1_floor_parallel duration:=5.0
```

시작 자세를 2초 유지한 뒤 구간당 `duration`초 동안 한 번 재생하고 최종 자세를 유지한다.
위치·회전을 보간하고 직전 관절각을 기준으로 IK를 계산한다. 전체 경로의 IK·관절 제한·인접 관절 변화 15° 제한을 통과해야 재생한다.
IK 실패 시 launch를 종료한다. 실제 충돌이나 동역학을 검사하는 기능은 없다.
`Goal flange axes`는 최종 목표 축이며, `pose_name:=init1_floor_parallel`을 정적 모드에 지정하면 목표만 표시한다.
여러 자세를 재생하려면 `pose_sequence:=init1,init1_floor_parallel,init1`을 사용한다.
자세 파일은 `coordinates_file`, 초기 추정 관절각은 `seed_file`로 변경할 수 있다.

## 실제 관절각 표시

`live_feedback`는 각 팔의 관절각을 20 Hz로 조회해 같은 모델에 표시한다. 기존 정적·동적 시각화 창은 Ctrl+C로 종료한 뒤 실행한다.
한 launch 안에서는 선택한 관절각 발행기 하나만 실행된다.
공식 `xarm-python-sdk`가 현재 ROS Python 환경에 설치되어 있어야 한다. 정적·동적 모드에는 SDK가 필요 없다.

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
ros2 launch simulation_validation init1.launch.py mode:=live_feedback
```

- 기본 주소는 왼팔 `192.168.2.31`, 오른팔 `192.168.2.32`다. `left_ip:=주소 right_ip:=주소`로 변경할 수 있다.
- 공식 SDK의 `get_joint_states(is_radian=True, num=1)`로 각 팔 J1~J7을 읽는다. ROS의 EEF pose 토픽은 사용하지 않는다.
- `/simulation_validation/joint_states` → `robot_state_publisher` → RViz로 연결한다. 장착 변환과 TCP는 기존 `coordinate_frames.yaml`을 사용한다.
- 이동, 모터 활성화, 모드·상태·TCP 설정 변경은 하지 않는다. 별도 제어기로 실물을 움직이면 화면이 따라간다.
- 팔별로 수신 여부를 터미널에 표시한다. 새 값이 1초 이상 없거나 조회에 실패하면 경고하고 마지막 자세를 유지한다. 연결 실패 시 2초 뒤 재시도한다.
- 최초 수신 전에는 해당 팔의 관절각을 발행하지 않는다. 한쪽만 수신되면 그 팔부터 표시하며, `init1`이나 영점 관절각으로 대체하지 않는다.
- 마지막 자세도 계속 표시하므로 `joint_states.header.stamp`는 화면 갱신 시각이다. 실제 수신 여부는 터미널 경고로 확인한다. 이 토픽은 시각화 전용이다.

화면 일치는 관절각뿐 아니라 장착 변환·기구 모델의 정확도에 좌우된다. `live_feedback`에서 base 장착값이나 TCP offset을 자동 보정하지 않는다.
기존 `mode:=init1` / `mode:=live` 대신 `mode:=static_rviz` / `mode:=live_feedback`을 사용한다.

## 검사

빌드 환경을 불러온 상태에서 실행한다. 로봇은 사용하지 않는다.

```bash
python3 -m unittest discover \
  -s main/control_pc/src/simulation_validation/test -p 'test_*.py' -v
```
