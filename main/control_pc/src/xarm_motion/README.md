# xarm_motion

C++ 독립 패키지다. 공통 YAML의 시작·목표 flange 자세를 읽고 공식 C++ SDK의 FK·IK를 기준으로 실물 경로를 계산한다. 공식 URDF는 관절 제한과 모델 차이 진단에 사용한다. 공식 C++ SDK로 관절 피드백을 받고 `set_servo_angle_j` 명령을 보낸다.

`simulation_validation`의 `live_control` 실행기로 실측 시각화와 함께 실행할 수 있다. `auto_align`은 아직 사용하지 않는다.

## 구성

- `motion_core`: 자세 읽기, 기구학 인터페이스, 위치·방향 보간, SDK FK 경로 저장, 시간 조정과 실행·추종·도착 검사다. 가짜 backend로 연결 없이 테스트할 수 있다.
- `xarm_backend`: 기존에 설치된 공식 SDK를 링크한다. 연결·피드백·FK·IK 조회·모드 전환·관절 명령·중지를 담당한다.
- `xarm_motion`: Enter 실행·Ctrl+C 중지를 처리하고 실제 관절·flange 피드백과 상태를 발행한다. ROS 실행·중지 서비스는 만들지 않는다.

## 빌드

공식 `xarm_description`이 기존 제어 PC 설치 경로에 준비되어 있어야 한다. SDK·공식 모델의 외부 소스는 수정하거나 다시 빌드하지 않는다.

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source ~/project/xarm_sdk/install/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 colcon --log-base main/control_pc/_colcon/log build \
  --base-paths main/control_pc/src \
  --packages-select xarm_motion \
  --build-base main/control_pc/_colcon/build \
  --install-base main/control_pc/_colcon/install \
  --symlink-install \
  --cmake-args -DBUILD_TESTING=ON
```

## 단독 실행

1. 기존 teleop·init·다른 로봇 제어기를 종료한다. 이 패키지가 실행 중일 때 다른 제어기를 시작하지 않는다.
2. 실제 로봇을 사용할 시작 자세에 놓는다. 이 패키지는 시작 자세로 자동 이동하지 않는다.
3. `main/control_pc/config/control.yaml`의 `execution.standalone_enabled`를 직접 `true`로 변경한다. 기본값은 `false`다.
4. 아래 명령에서 팔과 시작·목표 이름을 모두 지정한다.

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
  --target-pose init1_floor_parallel_v1
```

- `--arm`: `left`, `right`, `both` 중 필수 선택이다.
- 시작·목표는 `main/control_pc/config/arm_poses.yaml`의 이름이다. 목표를 v2로 바꾸려면 `--target-pose init1_floor_parallel_v2`를 지정한다.
- `READY` 후 **빈 줄 Enter**로 한 번 실행한다. 다른 문자는 실행하지 않는다.
- **Ctrl+C**는 진행 중인 모든 선택 팔에 SDK `set_state(4)` 중지를 요청하고 종료한다. 도착 시에도 중지 후 종료한다. 자동 재시작·자동 오류 해제는 없다.
- Enter 전에는 연결·조회·경로 계산만 한다. Enter 후 모터 활성화, servo mode 전환 및 이동 명령을 허용한다. TCP·payload·world offset·충돌 감도는 변경하지 않는다.
- 비활성화·설정 누락·잘못된 bool이면 SDK 객체 생성과 연결 전에 경고 후 종료한다. CLI·ROS 파라미터·환경 변수로 실행 허용 값을 덮어쓸 수 없다.
- 설정 파일은 빌드 시 고정한 저장소의 공통 `config/`를 읽는다. YAML만 변경한 경우 재시작하면 반영된다. 저장소 자체를 옮겼다면 다시 빌드한다.
- 실행·ROS 로그는 `_log/xarm_motion/`, 새 잠금 파일은 `_colcon/locks/xarm_motion/`에 둔다. 외부 native 잠금 파일이 이미 있으면 읽기 전용으로 열어 잠금을 함께 사용한다. 모든 외부 SDK 프로그램의 동시 접근을 막아 주는 장치는 아니다.

## 경로와 제한

실측 관절각을 첫 관절 명령의 기준으로 사용한다. 시작 자세 이름은 실측 flange가 허용 범위 안에 있는지 검사하는 용도다. SDK FK로 계산한 현재 flange에서 저장된 목표까지 계산하며, 시작점 IK로 관절각을 다시 맞추지 않는다. flange 목표에 고정 TCP offset을 곱하고 m를 mm로 바꿔 SDK에 전달한다.

위치는 직선, 방향은 quaternion 최단 회전으로 보간한다. SDK·URDF 관절 제한을 검사한다. SDK 1.17 IK에는 seed 인자가 없으므로, SDK FK의 수치 Jacobian으로 직전 관절 해 근처를 먼저 계산한다. 이 계산이 수렴하지 않으면 SDK IK 후보를 확인하고 연속성·FK 잔차 검사를 적용한다. 이 보정도 SDK FK만 사용하며 새 기구학 라이브러리는 추가하지 않는다. 해의 연속성과 SDK FK 잔차를 만족하지 못하면 이동 전에 거부한다. 관절해 사이에는 구간별 3차 보간을 적용하고, 전체 경로 시각에는 smoothstep을 적용해 시작·도착 속도를 0으로 한다. flange·TCP 속도와 관절 속도·가속도에 맞춰 전체 시간을 늘린다. 양팔은 더 긴 경로 시간에 맞춰 함께 출발·도착하도록 계산한다. 실행 중 실제 주기로 증분·가속도·TCP 속도를 다시 검사한다. 지연 시 양팔의 공통 경로 진행도를 줄여 제한을 만족하는 명령만 보내며, 가능한 진행도를 찾지 못하면 중지한다.

SDK FK·IK 조회는 PREPARING 단계에서 끝낸다. FK 경로는 중간점 오차 0.01 mm·0.0001 rad 기준으로 검사해 저장하고, 실행 중에는 저장한 경로를 보간한다. 100 Hz 제어 중 추가 FK·IK 조회는 하지 않는다. SDK 계산을 포함하므로 READY까지 준비 시간이 늘어날 수 있다.

`control.yaml`의 기본값 출처는 다음과 같다. 아래 외부 파일은 참조만 한다.

| 값 | 기본값 | 근거 |
|---|---|---|
| 주기·Cartesian 속도 | 100 Hz, 0.02 m/s, 0.2 rad/s | 외부 `teleop_xarm/omega7_cartesian_slave.cpp` |
| TCP 추종 오차 | 5 mm, 0.1 rad | 외부 `teleop_xarm/cartesian_follow.hpp` |
| 관절 증분·추종 오차 | `0.2 × min(dt, 0.004)` rad, 0.05 rad | 외부 `teleop_xarm/omega7_translation.hpp`의 `JointRamp` |
| 관절 가속도 | 25 deg/s² = 0.436332312999 rad/s² | 외부 `xarm_sdk/example/init.cc`; 새 관절 경로 제한에 적용 |

증분 수식은 100 Hz에서 최대 0.0008 rad/주기, 즉 0.08 rad/s에 해당한다. 경로 시간에는 지연 후 감속할 여유를 둔다. 기존 증분·가속도 제한에서 회복 가능한 속도의 최솟값에 20% 여유를 적용하므로 예상 이동 시간이 늘어날 수 있다. 경로 진행도는 실제 경과 시간과 분리하며, 지연 시간을 나중에 따라잡지 않는다. Servo API의 speed·acc 인자로 제한하는 방식이 아니다.

`validation`은 신규 초기값이며 실물 검증 전이다. 관절·TCP 조회는 100 ms, 약 200 ms 주기로 수신되는 rich 상태 보고는 별도 1초 타임아웃으로 검사한다. 시작 오차 5 mm·0.1 rad, 도착 오차 2 mm·0.02 rad·관절 0.005 rad, 정지 속도 0.01 rad/s를 사용한다. 도착 조건은 0.3초 유지하며 경로 진행 완료 후 최대 5초 확인한다. 지연으로 인한 무한 대기를 막기 위해 전체 실행은 READY 예상 시간의 2배와 도착 타임아웃의 합으로 제한한다. 기존 master 입력 타임아웃을 로봇 피드백 타임아웃으로 간주하지 않는다.

실제 SDK TCP offset이 `fixed_frames.yaml`의 flange → TCP와 다르거나 SDK world offset이 0이 아니면 거부한다. SDK FK와 실측 TCP의 차이는 추종 허용 범위 안이어야 한다. 실행 중 목표 오차를 임의 보정하거나 도착 허용값을 늘리지 않는다. `model1300`은 URDF 제한·진단 모델을 선택하며 실물 기구학은 SDK에서 계산한다.

책상·벽·손·마운트의 충돌 검사, 물리 시뮬레이션, 하드 실시간 제어는 제공하지 않는다. 이 패키지의 중지는 소프트웨어 요청이며 통신이 끊긴 로봇의 정지를 보장하는 물리 비상정지가 아니다. 경로 생성은 10,000구간·제한 적용 후 600초 안으로 제한한다.

## 피드백

- `/xarm_motion/joint_states`: 선택한 팔의 실측 관절각·속도.
- `/xarm_motion/left/base_flange_pose`, `/xarm_motion/right/base_flange_pose`: 실제 SDK TCP에서 고정 offset을 제거한 base 기준 flange 자세.
- `/xarm_motion/status`: 준비·대기·이동·완료·중지·오류 상태. 터미널에도 출력한다.

`MEASURE` 로그는 READY·이동 중 1초 간격·완료 시 양팔의 `tcp_pos_mm`, `tcp_rot_deg`, `joint_deg`, `speed_rad_s`를 `실측 오차/허용값[OK 또는 FAIL]`로 표시한다. 속도는 현재 관절 속도의 최댓값이다. `joint_deg`는 최종 목표 관절각과의 최대 차이다. 이동 중 FAIL은 아직 도착 조건을 만족하지 않았다는 뜻이다. 도착 시간 초과 메시지에도 마지막 측정값을 포함한다.

`model_sdk_mm`, `model_sdk_deg`는 같은 실측 관절각에 대한 로컬 FK와 SDK TCP의 차이다. `sdk_plan_mm`, `sdk_plan_deg`는 저장한 SDK 명령 경로와 실측 TCP의 차이다. 비교는 각 팔 base 기준이며, 실제 world 장착 좌표계의 정확성을 보장하지 않는다. 로그는 `main/control_pc/_log/xarm_motion/`에 저장한다.

`ros2 run simulation_validation live_control --arm both --start-pose init1 --target-pose init1_floor_parallel_v2`로 실측 시각화와 함께 실행한다. 실행 허용·시작 자세·Enter·중지 정책은 단독 실행과 같다. 완료 직전 마지막 실측값을 발행하며 시각화는 완료 상태로 마지막 수신 자세를 유지한다. 자세한 절차는 [제어 PC 시각화](../../../../docs/control_pc/README_simulation.md)를 따른다.

## 연결 없는 검증

```bash
cd ~/pipe_align_control_kkb
source /opt/ros/humble/setup.bash
source ~/project/xarm_sdk/install/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 colcon --log-base main/control_pc/_colcon/log test \
  --base-paths main/control_pc/src \
  --packages-select xarm_motion \
  --build-base main/control_pc/_colcon/build \
  --install-base main/control_pc/_colcon/install \
  --event-handlers console_direct+
colcon --log-base main/control_pc/_log/colcon test-result \
  --test-result-base main/control_pc/_colcon/build/xarm_motion --verbose
```

테스트는 공식 모델과 가짜 backend를 사용한다. 실제 SDK 연결·로봇 이동·외부 파일 수정은 하지 않는다.
