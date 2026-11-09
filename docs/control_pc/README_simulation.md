# 양팔 xArm 시각화 — 제어 PC

공식 xArm7 모델을 RViz에 표시해 실물과 양팔 base·flange·TCP 좌표계를 비교한다.
기본 `static_rviz`는 저장된 flange 자세 행렬을 표시한다. `dynamic_rviz`는 경로를 화면에서 재생하고, `live_feedback`은 실제 관절각을 표시한다.
빌드 후 아래 모드 중 하나를 선택한다. 실행 중인 시각화는 Ctrl+C로 종료한 뒤 다른 모드를 실행한다.

## 실행 모드

| 모드 | 동작 | 실물 연결 | 이동 명령 | 상태 |
|---|---|---|---|---|
| `static_rviz` | 이름으로 선택한 자세를 정적으로 표시 | 없음 | 없음 | 구현 |
| `dynamic_rviz` | 목표까지 이동 또는 여러 자세 사이의 경로를 화면에서 재생 | 없음 | 없음 | 구현 |
| `live_feedback` | 실제 양팔 관절 피드백을 화면에 반영 | 필요 | 없음 | 구현 |
| `live_control` | 실제 피드백 표시와 제어 명령 전송 | 필요 예정 | 있음 예정 | 미구현 예약 |

`live_control`을 선택하면 안내 오류와 함께 종료하며 실물에 접속하지 않는다.
기존 `mode:=init1`은 `mode:=static_rviz`로, `mode:=live`는 `mode:=live_feedback`으로 변경했다.

## 실행 환경

- 작업 경로: `~/pipe_align_control_kkb`.
- ROS2 Humble, `simulation_validation`, 공식 `xarm_description`을 사용한다.
- 빌드·설치·로그·공식 모델 캐시는 `main/control_pc/_colcon/`에 모은다.
- `live_feedback`에는 현재 ROS Python 환경의 공식 `xarm-python-sdk` 설치와 양팔 네트워크 연결이 필요하다.
- 아래 시각화 명령은 로봇 이동·모터 활성화·모드 변경을 하지 않는다. 실물 초기 자세 이동은 별도 항목이다.

## 1. 터미널 1 — 모델 준비와 빌드

공식 모델이 없는 경우에만 처음 한 번 내려받는다. 이미 준비되어 있으면 빌드부터 진행한다.

```bash
cd ~/pipe_align_control_kkb
mkdir -p main/control_pc/_colcon
touch main/control_pc/_colcon/COLCON_IGNORE
git clone --depth 1 --filter=blob:none --sparse --branch humble \
  https://github.com/xArm-Developer/xarm_ros2.git \
  main/control_pc/_colcon/dependencies/xarm_ros2
git -C main/control_pc/_colcon/dependencies/xarm_ros2 sparse-checkout set xarm_description
```

공식 모델과 시각화 패키지를 빌드한다. 외부 기존 제어 코드와 비전 PC 패키지는 빌드하지 않는다.

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
colcon --log-base main/control_pc/_colcon/log build \
  --base-paths main/control_pc/src \
    main/control_pc/_colcon/dependencies/xarm_ros2/xarm_description \
  --packages-select xarm_description simulation_validation \
  --build-base main/control_pc/_colcon/build \
  --install-base main/control_pc/_colcon/install \
  --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF
```

## 2. 터미널 1 — 정적 자세 표시

빌드를 마친 뒤 실행한다. 로봇 접속 없이 `control_coordinate.yaml`의 `init1` flange 행렬을 로컬 IK로 풀어 표시한다.
실물의 현재 자세와는 독립적이며, 실제 로봇을 초기 자세로 이동시키지 않는다.

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 launch simulation_validation init1.launch.py mode:=static_rviz pose_name:=init1
```

RViz에서 양팔 장착 방향·관절 배치·flange·TCP 축을 비교한다.
좌표축 X/Y/Z는 빨강/초록/파랑이며, 전체 관절 축은 `All link frames (optional)`을 켜서 확인한다.
이 터미널은 켜 둔다. 종료는 Ctrl+C다.

보정된 목표 자세만 정적으로 보려면 같은 환경에서 다음 명령으로 실행한다.

```bash
ros2 launch simulation_validation init1.launch.py mode:=static_rviz \
  pose_name:=init1_floor_parallel
```

## 3. 터미널 1 — 화면에서 이동 재생

실물에 접속하지 않고 `init1`에서 `init1_floor_parallel`까지의 경로를 계산한다.
시작 자세를 2초 표시한 뒤 5초 동안 한 번 재생하고 마지막 자세를 유지한다.

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 launch simulation_validation init1.launch.py mode:=dynamic_rviz \
  start_pose:=init1 target_pose:=init1_floor_parallel duration:=5.0
```

- flange 위치는 선형 보간하고 회전은 최단 회전으로 보간한다. 구간 시작·끝은 완만하게 재생한다.
- 이 두 자세는 flange 위치가 같으므로 화면에서 flange 중심을 유지하며 회전한다.
- 목표 flange 축은 `Goal flange axes`에 반투명 빨강/초록/파랑 화살표로 표시한다.
- 전체 경로를 먼저 계산한다. IK 실패·관절 제한 위반·인접 단계의 관절 변화가 15°를 넘으면 재생 없이 오류로 종료한다.
- 다시 재생하려면 Ctrl+C 후 같은 명령으로 실행한다. 실제 충돌이나 동역학을 검증하는 기능은 없다.

여러 자세를 연속 재생하려면 `pose_sequence`에 이름을 순서대로 넣는다. `duration`은 구간별 시간이다.
다음 예시는 보정 목표에 도달한 뒤 처음 자세로 돌아온다.

```bash
ros2 launch simulation_validation init1.launch.py mode:=dynamic_rviz \
  pose_sequence:=init1,init1_floor_parallel,init1 duration:=5.0
```

## 4. 터미널 1 — 실제 양팔 자세 표시

정적 표시 대신 실물 동기화를 사용할 때 실행한다. 기존 시각화 창은 먼저 종료한다.
각 팔 J1~J7을 공식 SDK로 20 Hz 조회해 RViz에 반영한다. teleop 실행 없이도 현재 자세를 조회할 수 있다.

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 launch simulation_validation init1.launch.py mode:=live_feedback
```

- 기본 주소는 왼팔 `192.168.2.31`, 오른팔 `192.168.2.32`다.
- 터미널에 양쪽 모두 `실제 관절각 수신 중`이 표시되는지 확인한다.
- 별도 제어기로 실물을 움직이면 화면이 따라간다. 시각화에서 실물로 이동 명령을 보내지 않는다.
- 수신 실패 또는 1초 이상 갱신 중단 시 경고하고 마지막 자세를 유지한다. 연결 실패 시 2초 뒤 재시도한다.
- 최초 수신 전에는 해당 팔의 자세를 표시하지 않는다. 한쪽만 수신되면 그 팔부터 표시한다.
- 화면이 멈춰 있으면 터미널의 수신 경고를 함께 확인한다. `init1` 자세로 자동 전환하지 않는다.

주소를 직접 지정하려면 실행 명령을 다음과 같이 바꾼다.

```bash
ros2 launch simulation_validation init1.launch.py mode:=live_feedback \
  left_ip:=192.168.2.31 right_ip:=192.168.2.32
```

## 5. 터미널 2 — 시각화 출력 확인

터미널 1의 시각화를 유지한 상태에서 새 터미널을 연다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 topic info -v /simulation_validation/joint_states
ros2 topic echo --once /simulation_validation/joint_states
ros2 topic hz /simulation_validation/joint_states
```

관절각은 rad이며, 양팔 수신 시 `left_joint1`~`left_joint7`, `right_joint1`~`right_joint7`의 14개 값이 출력된다.
정적 모드는 약 2 Hz, 동적 모드와 `live_feedback`은 약 20 Hz로 발행한다. 주기 확인은 Ctrl+C로 종료한다.
`live_feedback`은 마지막 수신 자세도 계속 발행하므로 토픽 주기와 timestamp만으로 실물 수신 여부를 판단하지 않는다. 터미널 1의 경고를 확인한다.

## 실물 init 1 비교 — 필요 시

실물을 `init 1`로 이동해 정적 화면과 비교하려는 경우에만 실행한다. **이 명령은 실제 양팔을 움직인다.**
[사전작업](../preliminary_work/README_control_pc.md)의 초기화 단계처럼 teleop 시작 전, Pipe를 잡지 않은 상태에서 실행한다.

```bash
source ~/project/xarm_sdk/install/setup.bash
cd ~/project/xarm_sdk/install/xarm_sdk/lib/xarm_sdk
./init 1
```

양팔의 `Successfully moved to initial pose.` 출력을 확인한다.
시각화만 실행할 때는 이 과정이 필요 없다.

## 표시 옵션과 설정

- GUI 없이 토픽·TF만 확인하려면 launch에 `rviz:=false`를 추가한다.
- 1300 계열 장착면 모델을 선택하려면 `model1300_left:=true model1300_right:=true`를 추가한다.
- 다른 좌표계 설정은 `frames_file:=/절대경로/coordinate_frames.yaml`로 지정한다.
- 자세 목록은 `coordinates_file:=/절대경로/control_coordinate.yaml`로 변경한다.
- 로컬 IK 초기 추정 관절각은 `seed_file:=/절대경로/init1.yaml`로 지정한다. 정적·동적 모드에서 사용한다.
- 공통 장착 변환과 TCP는 [coordinate_frames.yaml](../../main/control_pc/config/coordinate_frames.yaml)을 읽는다.
- 자세 행렬은 [control_coordinate.yaml](../../main/control_pc/config/control_coordinate.yaml)의 `poses` 아래에서 이름으로 선택한다. 각 자세는 `left/right.base_T_flange` 4×4 행렬이며, 각 팔 base 기준·위치 m 단위다.
- 현재 `init1`은 저장 당시 실측 자세, `init1_floor_parallel`은 flange 위치를 유지하면서 world -Z 방향을 보도록 계산한 목표다. world +Z가 수직이라는 가정을 사용했다.
- [init1.yaml](../../main/control_pc/src/simulation_validation/config/init1.yaml)의 관절각은 로컬 IK의 초기 추정값이다. 화면 자세 자체는 선택한 행렬로 결정한다. YAML 변경 후에는 시각화를 재시작한다.
- 토르소·손 형상과 미확인 카메라 장착 변환은 표시하지 않는다. 개체별 기구학 보정은 적용하지 않는다.

공식 URDF의 관절 원점·회전축·제한을 PyKDL에 연결해 로컬 FK·IK를 계산한다. 추가 라이브러리 설치는 필요하지 않았다.
7축 팔의 해는 유일하지 않으며 이전 관절각을 초기 추정값으로 사용한다. 최소 관절 변화의 전역 최적해를 보장하지는 않는다.
실측 행렬과 공식 모델 사이의 차이 때문에 `init1`을 IK로 표시한 관절각은 원래 초기 관절각과 최대 약 2.1° 다르다. `live_feedback`은 실측 관절각을 그대로 표시한다.

## 패키지 검사

시각화 코드 변경 후 실행한다. 실물 접속 없이 검사한다.

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
python3 -m unittest discover \
  -s main/control_pc/src/simulation_validation/test -p 'test_*.py' -v
```
