# auto_align

`pre_align_ready` 준비 이동과 카메라 좌표계 기반 `vision_align`(step4~7)을 C++로 실행한다. 비전 프로그램은 별도로 실행한다.

- `auto_align_node.cpp`: 파일 기반 실행 허용·Enter 실행·Ctrl+C 중지와 실측 피드백을 담당한다.
- `steps/step_01_vertical_move.cpp`: 수직 이동 목표와 `PreAlignReady`의 순서 연결·전체 경로 계산·양팔 시간 동기화를 담당한다.
- `steps/step_02_horizontal_move.cpp`: 방향·높이를 유지하는 수평 이동 목표를 계산한다.
- `steps/step_03_orientation_align.cpp`: 최종 방향 정렬 목표로 저장된 base 기준 자세를 사용한다.
- `collision_checker.cpp`: 공식 URDF 충돌 형상과 RViz 환경 YAML로 모델 충돌·여유거리를 검사한다.
- `steps/step_04_vision_measure.cpp`: 같은 영상의 좌우 측정, CAD 끝단·Z축 추출, 고정 목표와 입력 품질 검사.
- `steps/step_05_rotation_align.cpp`: 작은 base TCP 회전 시험, 국소 UVS Jacobian 추정·갱신과 방향 보정.
- `steps/step_06_translation_align.cpp`: 병진 시험과 끝단 위치 보정. 방향이 벗어나면 step5로 돌아간다.
- `steps/step_07_vision_verify.cpp`: 두 축·끝단·간격의 유지 검증과 단계 재진입.
- `vision_align_node.cpp`: 비전 ROS 입력과 기존 SDK 경로·실행기를 연결한다.

단계별 목표 함수는 `auto_align::pre_align_ready` 네임스페이스로 묶는다. 실행 순서는 파일명 정렬이 아니라 `PreAlignReady`가 01 → 02 → 03으로 명시한다. 세 단계는 기존 `--step pre_align_ready` 한 번으로 실행하며 로그의 `vertical` → `horizontal` → `orientation` 표시는 유지한다.

## 동작

양팔을 함께 사용하며 시작 자세 이름은 받지 않는다. `control.yaml`의 `pre_align_ready.target_pose`로 `arm_poses.yaml`의 목표 이름을 선택한다. 현재 선택은 `pre_align_ready_v2`이며 기존 `pre_align_ready`도 보존한다.

```yaml
pre_align_ready:
  target_pose: pre_align_ready_v2
```

기존 `pre_align_ready` 아래의 충돌 검사 설정과 함께 사용한다. 다른 자세를 추가한 뒤 목표 이름만 바꾸면 같은 세 단계로 이동한다. 누락·미등록 목표·잘못된 양팔 행렬은 로봇 연결 전에 거부한다. 실행 시 목표를 읽어 고정하므로 변경 후에는 실행기를 다시 시작한다. YAML 변경만으로 재빌드할 필요는 없다.
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

RViz와 제어기는 같은 설정 목표를 사용하며 `READY`에 선택된 이름을 표시한다. 시각화 시작과 제어기 시작 사이에 `control.yaml` 또는 `arm_poses.yaml` 내용이 바뀌면 연결 전에 거부한다. 내부 `--expected-inputs` 인자는 이 일치 확인용이며 목표나 실행 허용을 덮어쓰지 않는다.

`READY` 후 빈 줄 Enter로 한 번 실행한다. 경로 계산 후 실제 관절각이 바뀌면 거부한다.
오류·Ctrl+C·완료 시 양팔에 중지를 요청한다. 오류 원인은 SDK 연결 해제 대기 전에 발행하며, 시각화도 해당 원인을 표시한다. 가능한 명령을 찾지 못하면 관절 번호·증분/가속도·허용값·실제 주기를 기록하고 중지한다. FoundationPose는 자동 실행하지 않는다.
로그는 `main/control_pc/_log/auto_align/`, 잠금은 기존 `xarm_motion`과 공통 `_colcon/locks/xarm_motion/`에 둔다.
실측 토픽은 `/auto_align/joint_states`, `/auto_align/{left,right}/base_flange_pose`, `/auto_align/status`다.

사용 후 실행 허용을 `false`로 되돌린다. 로봇 없는 검증은 가짜 backend와 FCL 형상을 사용한다. 실물 실행은 사용자가 직접 수행한다.

## Step4~7 비전 UVS

`pre_align_ready_v2`로 준비한 상태에서 별도로 시작한다. 준비 이동을 다시 실행하지 않는다.
카메라 optical X축을 양쪽 파이프의 공통 목표 방향으로 사용한다. CAD Z축이 파이프 축이며,
`docs/vision_pc/pipe1.obj`의 Z=0 / 316 mm 끝단 중 보이는 플랜지를 설정한다.
현재 양팔의 끝단 후보는 Z=316 mm다. `vision_align.left_endpoint_z_m` / `right_endpoint_z_m`로
바꿀 수 있다. 양쪽 끝단의 바깥 법선이 서로 마주보지 않으면 실행을 거부한다.

ROS 환경은 위 실행과 같다. 측정 전용은 로봇 SDK 객체를 만들거나 연결하지 않으며 Enter도 요구하지 않는다.

```bash
ros2 run auto_align auto_align --step vision_measure
```

`/auto_align/vision_diagnostics`에 끝단 위치·CAD 끝단·각도·상대 Y/Z 차이·영상 나이를 발행한다.
`/auto_align/vision_markers`에는 양쪽 축(초록/파랑)과 목표 X축(노랑)을 카메라 frame으로 발행한다.
RViz에서 이를 보려면 Fixed Frame을 `camera_left_optical_frame`으로 하고 MarkerArray display를 추가한다.
카메라 외부 캘리브레이션이 없으므로 world와 카메라 사이 TF를 임의로 만들지 않는다.
이 표시는 3D 축 표시이며 카메라 영상 위의 픽셀 overlay는 아니다. 실영상의 플랜지 대응도 확인한다.
측정 전용 상태는 `/auto_align/vision_status`를 사용해 기존 제어 상태와 섞이지 않게 한다. Ctrl+C로 종료한다.

실제 UVS는 teleop 및 다른 팔 제어기를 종료한 뒤 `control.yaml`의
`execution.auto_align_enabled`와 **`execution.vision_align_enabled`를 모두 true**로 설정한다.
새 허용값의 기본은 false다. CLI나 ROS 파라미터로 우회하지 않는다.

```bash
ros2 run auto_align auto_align --step vision_align
# 로봇 실측 RViz 표시를 함께 사용하려면 위 명령 대신:
ros2 run simulation_validation live_control --arm both --step vision_align
```

READY 후 Enter는 양팔의 작은 시험 이동을 허용한다. 회전 및 병진을 학습할 때 팔별 3개 base축으로
양의 시험 이동을 하고, 실측 변위로 돌아온다. 정밀 camera–base 및 grasp 변환은 입력하지 않는다.
SDK FK·IK, URDF 관절 제한, 기존 전체 경로 충돌 검사 후에만 보정 경로를 실행한다.
회전은 base 기준 TCP를 중심으로 하므로 끝단 이동을 포함한 전체 반응을 카메라에서 다시 측정한다.
양팔의 보정은 서로 영향을 구분할 수 있도록 한 번에 한 팔씩 수행한다.

작은 정지 관절각 변동은 실측 자세에서 기존 경로로 연결하는 별도 구간으로 처리한다. 이 연결도
SDK FK·속도·가속도 및 전체 모델 충돌 검사를 통과해야 한다. `START_CONNECTION`에는 재검사와 연결 시간 진단을
표시한다. 가속도 허용값을 늘리거나 기존 검사 경로의 시작 관절각을 임의로 치환하지 않는다.

- step4: 정지 측정으로 목표 Y/Z 중간값과 각 끝단의 초기 X를 고정한다.
- step5: 국소 Jacobian으로 축 방향을 X축에 맞추며 반복 검증한다. 원통 축 주위 roll은 제어하지 않는다.
- step6: 끝단 위치를 보정한다. 목표 Y/Z로 이동하면서 초기 X도 유지하도록 제어한다. 방향이 틀어지면 step5로 돌아간다.
- step7: 개별·상대 각도와 끝단 위치를 확인한다. 통과하면 여러 독립 측정창에서 유지 후 완료한다.

설정은 `vision_align` 아래에 모았다. 각도 2°, 개별 끝단 위치 3 mm(초기 X 유지 포함), 상대 Y/Z 5 mm가 초기값이다.
완료에는 최소 1.5초와 5개의 서로 다른 정지 측정 결과가 모두 필요하다. 각 정지 측정은 최소 0.8초·3쌍을
평균하므로 실제 완료 확인은 이보다 길 수 있다. 3 Hz 이상 가변 입력을 고정 프레임 수로 제한하지 않는다.
측정창 내 위치 RMS 2.5 mm·축 각도 RMS 1°를 넘으면 거부한다. 이는 정확도의 보증값이 아니다.

동일 timestamp 좌우 쌍만 사용하고 중복·역순·한쪽 누락을 처리한다. 이동이 멈춘 뒤 0.2초 이후의
영상으로만 보정을 검증한다. PC 시각 동기화가 필요하며 영상 나이 1초, 좌우 쌍 수신 공백 1.5초를
각각 검사한다. 평균 주기만 3 Hz 이상이고 영상이 오래된 상황은 허용하지 않는다.
입력 감시는 경로 계산·실행·측정 중 계속 유지한다. 영상 끊김, 축 반전, 오차 악화, 식별 불가,
이동 범위·끝단 간격·시간·반복 상한 초과 시 중지한다.

기본 시험량은 병진 3 mm·회전 1.5°, 반복 보정 상한은 2 mm·1°다. 비전용 Cartesian 상한은
5 mm/s·0.05 rad/s이며 기존 관절 증분·가속도 제한도 적용한다. 전체 한계는 끝단/TCP 이동 60 mm,
회전 약 20°, 최소 카메라 X 끝단 간격 30 mm·초기 간격 변화 10 mm, 120회 이동·300초다.
카메라·토르소 고정과 파이프가 손 안에서 미끄러지지 않는 조건에서 준비 자세 주변을 대상으로 한다.

카메라 기반 간격 감시는 손·파이프 전체 충돌 검사를 대신하지 않는다. 기존 충돌 모델은 팔·환경만
포함하고 장착 변환 오차도 남아 있다. 실물의 손·파이프 및 환경 여유는 별도로 확인해야 한다.
현재는 실제 하드웨어에서 학습·정렬을 검증하지 않았다.

테스트는 미지의 서로 다른 양팔 base와 회전/끝단 결합을 가진 가짜 장면의 수렴, 3/10/25 Hz,
노이즈와 병진 후 방향 재보정, 중복 영상·축 반전·학습 실패·상한을 확인한다.
ROS 측정 전용 통합 테스트는 별도 domain의 가짜 3/12 Hz publisher만 사용하고 SDK에 연결하지 않는다.
