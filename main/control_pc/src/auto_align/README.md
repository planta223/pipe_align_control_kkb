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
- `vision_cache.cpp`: 팔별·단계별 Jacobian, 기준 자세·노이즈·품질 저장과 조건 확인.

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

`pre_align_ready`의 SDK 경로 계산은 팔별 독립 backend에서 병렬로 수행한다. 각 팔의 세 구간은
순차 계산하고, 양쪽 작업이 모두 끝난 뒤 구간 시간을 동기화한다. 실패 시 두 작업을 합류시킨 후
오류를 전달하며, 준비 중 모터 활성화나 이동 명령은 보내지 않는다.
충돌 FK는 부모부터 링크별로 한 번 계산해 형상들이 공유한다. 시점 사이 이동량은 관절별 속도 상한과
형상에서 각 조상 관절까지의 회전 반경 상한을 합산한다. 같은 팔의 두 형상을 함께 움직이는 공통 조상
회전은 상대 거리 변화에서 제외한다. 점 검사·구간 재분할·20 mm 여유거리 조건은 유지한다.

로그의 `PERF:` 항목으로 다음을 비교할 수 있다.

- `INPUTS`: 시작 관절각, 제어·목표 및 환경 설정 해시, 속도·가속도와 자동 시작 여부.
- `SDK_PLAN`: 전체 준비 시간, 팔별 시간·구간별 시간, FK 호출 횟수·시간, 수치 IK 반복 및 SDK IK fallback 횟수·시간.
  `solve_s`는 내부 FK 시간을 포함하므로 `fk_s`와 더하지 않는다. FK 시간에는 backend의 관절 제한 조회도 포함된다.
- `COLLISION`: 검사 시간, 샘플·링크 FK·형상 쌍·AABB 제외·정밀 충돌/거리 조회 횟수, 재분할 횟수·최대 깊이.
  `fk_transform_s`는 FK·형상 변환·AABB 갱신 시간, `narrowphase_s`는 정밀 검사 시간을 나타낸다.

저장소 루트에서 기존 SDK·공식 모델 환경을 불러온다. 외부 소스는 수정하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/project/xarm_sdk/install/setup.bash
source main/control_pc/_colcon/install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 colcon --log-base main/control_pc/_colcon/log build \
  --base-paths main/control_pc/src --packages-select xarm_motion auto_align simulation_validation \
  --build-base main/control_pc/_colcon/build --install-base main/control_pc/_colcon/install \
  --symlink-install --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

빌드 유형을 지정하지 않으면 `RelWithDebInfo`(최적화·디버그 심볼 포함)를 기본으로 사용한다.
명시적으로 지정한 `Debug` 등 다른 빌드 유형은 유지한다.

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
Enter 대기를 생략하려면 `pre_align_ready` 또는 `vision_align` 명령 뒤에 `--auto-start`를 붙인다.
기본값은 수동 시작이며, 이 옵션은 준비·충돌 검사·실행 허용·시작 자세 검사를 생략하지 않는다.
Ctrl+C/SIGINT 중지와 터미널 요구는 유지한다. `vision_measure`에는 사용할 수 없다.
RViz 실행 래퍼도 `ros2 run simulation_validation live_control --arm both --step pre_align_ready --auto-start`처럼 전달한다.
허용 범위 안의 작은 실측 차이는 기존 시작점까지 연결 경로를 추가한다. 이때 새 연결 구간만
충돌 재검사하고, 관절 경로가 그대로인 기존 구간은 같은 실행에서 고정된 환경·충돌 설정으로
최초 검사한 결과를 재사용한다. 연결 구간 검사 후 자세가 바뀌면 활성화 전에 거부한다.
오류·Ctrl+C·완료 시 양팔에 중지를 요청한다. 오류 원인은 SDK 연결 해제 대기 전에 발행하며, 시각화도 해당 원인을 표시한다. 가능한 명령을 찾지 못하면 관절 번호·증분/가속도·허용값·실제 주기를 기록하고 중지한다. FoundationPose는 자동 실행하지 않는다.
로그는 `main/control_pc/_log/auto_align/`, 잠금은 기존 `xarm_motion`과 공통 `_colcon/locks/xarm_motion/`에 둔다.
실측 토픽은 `/auto_align/joint_states`, `/auto_align/{left,right}/base_flange_pose`, `/auto_align/status`다.

사용 후 실행 허용을 `false`로 되돌린다. 로봇 없는 검증은 가짜 backend와 FCL 형상을 사용한다. 실물 실행은 사용자가 직접 수행한다.

## Step4~7 비전 UVS

`pre_align_ready_v2`로 준비한 상태에서 별도로 시작한다. 준비 이동을 다시 실행하지 않는다.
카메라 optical X축을 양쪽 파이프의 공통 목표 방향으로 사용한다. CAD Z축이 파이프 축이며,
`docs/vision_pc/pipe1.obj`의 Z=0 / 316 mm 끝단 중 보이는 플랜지를 설정한다.
`docs/vision_pc/20261119_170348.jpg`의 빨간 원으로 확인한 양팔 안쪽 면은 현재 추적 자세에서 Z=0 mm다.
`vision_align.left_endpoint_z_m` / `right_endpoint_z_m`로
바꿀 수 있다. 양쪽 끝단의 바깥 법선이 서로 마주보지 않으면 실행을 거부한다.

ROS 환경은 위 실행과 같다. 측정 전용은 로봇 SDK 객체를 만들거나 연결하지 않으며 Enter도 요구하지 않는다.

```bash
ros2 run auto_align auto_align --step vision_measure
```

`/auto_align/vision_diagnostics`에 끝단 위치·CAD 끝단·각도·상대 Y/Z 차이·영상 나이를 발행한다.
같은 `VISION:` 로그에 카메라 좌표계 이름(`frame_id`), 좌우 CAD Z축 단위 벡터(`axis_camera=[x y z]`),
축 방향 정규화 부호(`axis_sign`), 부호 있는 `angle_xy_deg`·`angle_xz_deg`도 기록한다.
`a = axis_sign * axis_camera`에 대해 각각 `atan2(a.y, a.x)`·`atan2(a.z, a.x)`를 도 단위로 표시한다.
정렬 중에는 제어 시작 평균 측정의 ±X 부호를 사용하며, 읽기 전용 `vision_measure`에서는 각 측정의 ±X 부호를 사용한다.
기존 `axis_X_deg`와 `relative_angle_deg`는 평행도 판정을 위한 부호 없는 각도 크기로 유지한다.
축 벡터가 없는 과거 로그에서는 각도 부호를 복원할 수 없다.
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

READY 후 Enter는 양팔의 작은 시험·보정 이동을 허용한다. 회전 및 병진을 새로 학습할 때 팔별 3개 base축으로
양의 시험 이동을 연속 실행하며 복귀하지 않는다. 각 시험의 직전·직후 영상 차이와 실제 SDK 변위로
대응 관계를 구한다. 기본은 단계별 양팔 6회이며 회전·병진을 모두 새로 학습하면 12회다.
회전은 정보 부족·응답 불일치·정체 시 팔당 3° 시험을 최대 1회 추가해, 실행 전체에서 최대 8회다.
따라서 병진 학습까지 포함하면 최대 14회이며 필요한 보정 이동은 별도다. 재학습도 이 예산을 공유한다.
이미 조건을 만족한 팔·단계의 학습은 생략한다. 정밀 camera–base 및 grasp 변환은 입력하지 않는다.
SDK FK·IK, URDF 관절 제한, 기존 전체 경로 충돌 검사 후에만 보정 경로를 실행한다.
회전은 base 기준 TCP를 중심으로 하므로 끝단 이동을 포함한 전체 반응을 카메라에서 다시 측정한다.
양팔의 보정은 서로 영향을 구분할 수 있도록 한 번에 한 팔씩 수행한다.

### 학습값 저장·검증·재사용

`control.yaml`의 `vision_align.cache.enabled: true`이면
`main/control_pc/_state/vision_align/jacobians.yaml`을 사용한다. 이 실행 상태 파일은 Git에서 제외한다.
저장값이 없거나 유효하지 않으면 신규 학습한다. 읽기 전용 `vision_measure`는 저장값을 읽거나 쓰지 않는다.

- 저장값은 왼팔/오른팔 × 회전/병진으로 분리한다. Jacobian, 기준 끝단·CAD 축·base TCP 자세,
  저장 시각, 행렬 조건수·측정 불확실성을 기록한다. 초기 X와 목표 Y/Z, 누적 이동 기준은 매 실행 새 측정으로 정한다.
- 로봇 IP·모델/URDF, fixed frames/TCP, 비전 토픽·frame·파이프 길이·CAD 끝단, 로컬 `pipe1.obj`,
  `cache.context_id`가 달라지면 저장값을 사용하지 않는다. 속도·gain만 바꾼 경우에는 유지할 수 있다.
- 기본 유효기간은 24시간이다. 현재 끝단/TCP 위치가 저장 기준에서 30 mm 이내이고,
  파이프 축/TCP 방향 차이가 0.15 rad 이내여야 재사용 후보가 된다.
- 후보로 계산한 첫 보정부터 **모든 재사용 보정**에서 `J × 실제 SDK 변위`와 영상 변화를 비교한다.
  별도 왕복 시험 대신 필요한 보정 이동을 검증에도 사용한다. 한 번 통과해도 전체 방향이 검증된 것은 아니다.
- 신규 학습으로 계산한 보정에도 같은 검사를 적용한다. 기본 시험에서는 실제 입력의 3축 독립성과,
  불확실성으로 정규화한 반응 행렬의 최소 특잇값을 검사한다. 회전은 관측 가능한 2차원, 병진은 3차원을 요구한다.
  파이프 축 주위 roll의 영상 반응이 작다는 이유만으로 거부하지 않는다. `minimum_response_snr: 3.0`은
  비교 지표의 한도이며 통계적 신뢰확률을 뜻하지 않는다. 시험 데이터는 불확실성에 따라 가중해 추정한다.
- 위치(m)와 각도(rad)는 별도로 비교한다. 활성 작업의 예측·실측 신호가 불확실성의 3배 이상이어야 하며,
  방향 일치·크기 비율 0.5~1.5·잔차 한계를 확인한다. 회전 보정은 각도, 병진 보정은 위치로 판정한다.
  비활성 성분의 결합 변화도 로그에 남기고 기존 이동 범위·단계 전환·최종 검증으로 확인한다.
  불확실성은 평균의 표준오차와 측정창 시작/끝 변화의 절반 중 큰 값이며, 설정한 노이즈 하한도 적용한다.
  영상들이 서로 독립이라는 가정만으로 drift를 작게 평가하지 않는다.
- 신호가 부족하면 온라인 행렬 갱신을 보류한다. 기본 `max_untrusted_responses: 3`회 연속 저신호 또는
  불확실성보다 작은 개선이 이어지면 해당 팔·단계만 현재 위치에서 재추정한다. 명확한 응답 불일치는 즉시 재추정한다.
  저장 후보만 사용했던 단계는 기본 3축 시험을 수행하고, 이미 시험한 회전 단계는 남은 추가 시험 1회만 사용한다.
  추가 회전 방향은 정보가 부족한 작업 방향으로 선택한다. 예산 소진 또는 재시험 후에도 품질 부족이면 중지한다.
  충돌·오래된 영상·추적 중단·이동/시간 상한은 재학습으로 우회하지 않고 중지한다.
- step7 유지 검증에 성공한 실행에서만 학습값을 갱신한다. 미사용 저장 항목은 기존 시각을 유지한다.
  끝단이 우연히 목표 안에 들어가도 응답을 한 번도 검증하지 못한 저장 후보는 새 시각·자세로 갱신하지 않는다.
  임시 파일 기록·동기화 후 교체하므로 실패·중단한 실행은 기존 정상값을 덮어쓰지 않는다.
  파일 저장 자체의 실패는 `CACHE: save_failed`로 남기며 정렬 결과와 구분한다.

카메라를 물리적으로 옮기거나, 외부 캘리브레이션·추적 모델·파이프 grasp가 바뀐 것은 로컬 설정만으로
완전히 식별할 수 없다. 이때 `cache.context_id`를 바꾸거나 다음 명령으로 강제 재학습한다.
강제 재학습도 이미 정렬된 팔·단계에 불필요한 시험을 추가하지 않으며, 실행 허용과 안전 검사를 유지한다.

```bash
ros2 run auto_align auto_align --step vision_align --relearn
# 준비 후 Enter 없이 실행하려면 --auto-start를 추가한다.
ros2 run simulation_validation live_control --arm both --step vision_align --relearn
```

`CACHE:`는 읽기·조건 확인·저장을, `CACHE_VALIDATE:`는 저장 후보의 방향별 검증 결과를 기록한다.
`UVS_PROBE_RESPONSE:`는 실제 입력과 관측 변화, `UVS_LEARN_QUALITY:`는 정보량·행렬 품질·행렬값을 남긴다.
`UVS_RESPONSE:`에는 신규/저장값 구분, 요청·실측 입력, 예측·관측 변화, 위치/각도 불확실성이 함께 남는다.
`VISION_MOVES:`는 팔별·단계별 학습·보정·검증 횟수를 남긴다. **검증 횟수는 보정 횟수의 부분집합**이므로
전체 이동은 학습+보정이며 검증을 다시 더하지 않는다. `PROBE:`에서 신규 학습 축을 확인할 수 있다.
`VISION:`의 `mean_position_error_m` / `mean_angle_error_rad`는 검증에 쓰는 정지 평균 불확실성이다.

### 대기 시간과 상세 계측

이동 직후 이미 얻은 새 정지 평균 측정은 다음 반복에서 재사용한다. 이동 없이 새 측정을 얻지 못한 반복만
별도로 측정하므로 중복 평균 대기를 줄인다. step7의 독립 측정창 유지 검증은 보존한다.
평균창 0.8초·이동 후 안정화 0.2초·도착 유지 0.3초·반복 회전 상한 1°는 유지한다.

회전 경로에는 `rotation_linear_speed_m_s: 0.01`, 병진에는 `linear_speed_m_s: 0.005`를 적용한다.
회전 경로도 TCP offset 때문에 선속도 항의 영향을 받는다. 공통 `limits`의 더 낮은 상한은 그대로 적용하며,
공유 motion core의 최소 시간·관절 증분·가속도·충돌 검사는 변경하지 않는다.

- `VISION_CONFIG:`: 해당 실행의 vision/limits/validation 설정 스냅샷.
- `VISION_MOVE_REQUEST:`: 회차, 팔, 동작 종류, 요청 벡터.
- `PERF: VISION_MOVE`: 팔별 SDK 경로 계산, 최초 충돌 검사, 준비·시작 연결, 실행 시간, 계획 시간,
  적용 속도, 요청/SDK 실측 벡터와 차이. `execute_including_hold_s`는 도착 유지 시간을 포함한다.
  `begin_s`는 시작 연결 재검사를 포함하므로 별도 충돌 재검사 시간을 다시 더하지 않는다.
- `PERF: VISION_MEASURE`: 첫 유효 영상 대기, 평균 수집, 전체 시간, 샘플 수와 양팔 위치·각도 RMS.
  품질 실패 시에도 양팔 RMS·각 한도·측정창 stamp를 기록한다. RMS는 평균 불확실성과 다른 값이다.

이번 변경은 학습 알고리즘 버전을 갱신하므로 이전 버전 저장값은 다음 실행에서 사용하지 않는다.

측정 전용으로 CAD 축과 선택 끝단도 확인한다. 예를 들어 좌측 끝단 X가 우측보다 작은데 좌측 CAD 축은 −X,
우측 CAD 축은 +X이고 두 `endpoint_z_m`가 모두 파이프 길이라면 바깥 법선이 서로 반대로 향한다.
이 조합은 기존 끝단 검사에서 거부한다. 실제 마주보는 플랜지를 영상으로 확인한 뒤 끝단 설정을 정한다.

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

기본 시험량은 병진 3 mm·회전 2°(추가 시험 3°), 반복 보정 상한은 2 mm·1°다. 비전용 상한은
병진 5 mm/s·회전 경로용 선속도 10 mm/s·각속도 0.05 rad/s이며 기존 관절 증분·가속도 제한도 적용한다. 전체 한계는 끝단/TCP 이동 60 mm,
회전 약 20°, 최소 카메라 X 끝단 간격 30 mm·초기 간격 변화 10 mm, 120회 이동·300초다.
카메라·토르소 고정과 파이프가 손 안에서 미끄러지지 않는 조건에서 준비 자세 주변을 대상으로 한다.

카메라 기반 간격 감시는 손·파이프 전체 충돌 검사를 대신하지 않는다. 기존 충돌 모델은 팔·환경만
포함하고 장착 변환 오차도 남아 있다. 실물의 손·파이프 및 환경 여유는 별도로 확인해야 한다.
현재는 실제 하드웨어에서 학습·정렬을 검증하지 않았다.

테스트는 미지의 서로 다른 양팔 base와 회전/끝단 결합을 가진 가짜 장면의 수렴, 3/10/25 Hz,
노이즈와 병진 후 방향 재보정, 중복 영상·축 반전·학습 실패·상한을 확인한다.
ROS 측정 전용 통합 테스트는 별도 domain의 가짜 3/12 Hz publisher만 사용하고 SDK에 연결하지 않는다.
