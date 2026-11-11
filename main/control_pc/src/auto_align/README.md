# auto_align

첫 단계인 `pre_align_ready`를 C++로 실행한다. FoundationPose 실행과 미세 정렬은 아직 연결하지 않았다.

- `auto_align_node.cpp`: 파일 기반 실행 허용·Enter 실행·Ctrl+C 중지와 실측 피드백을 담당한다.
- `steps/step_01_vertical_move.cpp`: 수직 이동 목표와 `PreAlignReady`의 순서 연결·전체 경로 계산·양팔 시간 동기화를 담당한다.
- `steps/step_02_horizontal_move.cpp`: 방향·높이를 유지하는 수평 이동 목표를 계산한다.
- `steps/step_03_orientation_align.cpp`: 최종 방향 정렬 목표로 저장된 base 기준 자세를 사용한다.
- `collision_checker.cpp`: 공식 URDF 충돌 형상과 RViz 환경 YAML로 모델 충돌·여유거리를 검사한다.

단계별 목표 함수는 `auto_align::pre_align_ready` 네임스페이스로 묶는다. 실행 순서는 파일명 정렬이 아니라 `PreAlignReady`가 01 → 02 → 03으로 명시한다. 세 단계는 기존 `--step pre_align_ready` 한 번으로 실행하며 로그의 `vertical` → `horizontal` → `orientation` 표시는 유지한다.

## 동작

양팔을 함께 사용하며 시작 자세 이름은 받지 않는다. `arm_poses.yaml`의 `pre_align_ready`가 목표다.
각 팔의 world X·Y와 flange 방향을 유지하며 해당 팔의 목표 높이로 이동하고, 방향을 유지하며 목표 X·Y로 이동한 뒤 최단 회전으로 목표 방향에 맞춘다.
각 구간은 양팔 중 더 긴 시간에 맞추며, 구간 사이에는 기존 도착 유지 시간만큼 목표 관절각을 유지한다. 최종 도착은 기존 위치·방향·관절·정지 속도 조건으로 확인한다.

전체 SDK FK·IK·기본 시간 조정과 충돌 검사는 Enter 전에 수행한다. 실행 중에는 저장한 경로와 기존 100 Hz 제한 검사를 사용한다. 제어 주기가 늦어지면 양팔의 공통 경로 진행도를 줄이며, 관절별로 경로를 변경하지 않는다. 지연 후 감속 여유를 위해 이동 시간이 늘어날 수 있으며 READY에는 이를 반영한 예상 시간을 표시한다. 단계 표시도 실제 경과 시간이 아닌 경로 진행도를 따른다. 수직 이동은 목표 높이에 따라 상승 또는 하강한다. 충돌 경로를 자동 우회하지 않는다.

충돌 대상은 팔의 자기 충돌·양팔 사이·활성화된 책상·블라인드·오른쪽 벽이다. 고정 연결과 바로 인접한 관절 링크는 자기 충돌 검사에서 제외한다. 손·파이프·토르소 형상은 포함하지 않는다.
`simulation_validation/config/environment_models.yaml`의 크기·위치·표시용 상자 중심과 같은 배치를 사용한다. `show_environment` 같은 RViz 실행 옵션은 검사에 영향을 주지 않는다. 환경 YAML의 `enabled`는 해당 물체의 모델 포함 여부를 결정한다.

20 mm는 모델 형상 사이의 최소 거리다. 메시 거리와 관절 속도에서 얻은 이동량 상한으로 시점 사이도 검사하며, 최소 간격까지 세분해도 여유거리를 확인하지 못하면 거부한다. 실제 배치 오차·기구학 차이·추종 오차 및 모델에 없는 물체의 충돌을 보증하는 값은 아니다.

## 빌드

저장소 루트에서 기존 SDK·공식 모델 환경을 불러온다. 외부 소스는 수정하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/project/xarm_sdk/install/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 colcon --log-base main/control_pc/_colcon/log build \
  --base-paths main/control_pc/src --packages-select xarm_motion auto_align simulation_validation \
  --build-base main/control_pc/_colcon/build --install-base main/control_pc/_colcon/install \
  --symlink-install --cmake-args -DBUILD_TESTING=ON
```

## 실행

teleop·init·다른 로봇 제어기를 종료하고 손의 grasp 상태는 유지한다. `control.yaml`의 `execution.auto_align_enabled`를 파일에서 직접 `true`로 바꾼다. 기본값은 `false`이며 CLI·ROS 파라미터·환경 변수로 우회할 수 없다. `standalone_enabled`와는 별도다.

```bash
source /opt/ros/humble/setup.bash
source ~/project/xarm_sdk/install/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 run auto_align auto_align --step pre_align_ready
```

RViz와 함께 실행하려면 같은 환경에서 위 마지막 명령 대신 다음을 사용한다. 시작·목표 자세 인자는 받지 않는다.

```bash
ros2 run simulation_validation live_control --arm both --step pre_align_ready
```

완료 후 RViz는 마지막 수신 자세를 유지한다. Ctrl+C로 닫는다.

`READY` 후 빈 줄 Enter로 한 번 실행한다. 경로 계산 후 실제 관절각이 바뀌면 거부한다.
오류·Ctrl+C·완료 시 양팔에 중지를 요청한다. 오류 원인은 SDK 연결 해제 대기 전에 발행하며, 시각화도 해당 원인을 표시한다. 가능한 명령을 찾지 못하면 관절 번호·증분/가속도·허용값·실제 주기를 기록하고 중지한다. FoundationPose는 자동 실행하지 않는다.
로그는 `main/control_pc/_log/auto_align/`, 잠금은 기존 `xarm_motion`과 공통 `_colcon/locks/xarm_motion/`에 둔다.
실측 토픽은 `/auto_align/joint_states`, `/auto_align/{left,right}/base_flange_pose`, `/auto_align/status`다.

사용 후 실행 허용을 `false`로 되돌린다. 로봇 없는 검증은 가짜 backend와 FCL 형상을 사용한다. 실물 실행은 사용자가 직접 수행한다.
