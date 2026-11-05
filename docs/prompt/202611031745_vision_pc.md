# 비전 PC Codex 인수인계

이 저장소는 Control PC에서 개발한 `pipe_align_control_kkb`를 비전 PC에 clone한 것이다. 아래 맥락을 유지하여 비전 PC 작업을 이어가라.

## 목표와 작업 순서

- 목표: FoundationPose의 두 Pipe 6D pose를 이용한 Dual xArm 자율 정렬. 목표 일정은 2026-11-09~10이다.
- 먼저 제어 PC에서 Omega7/xArm 초기화 → teleop → 두 Pipe grasping → 기본 자세 준비를 마친다.
- **그다음 비전 PC에서 ZED2i와 FoundationPose를 실행한다.** 이 작업의 시작 조건은 grasping 완료다. 자동 grasping이나 공유제어는 이번 개발 범위가 아니다.
- Omega7과 자율제어를 결합하는 공유제어는 연구실 선배가 추후 담당한다. AnyGrasp를 자율 정렬의 필수 경로로 추가하지 않는다.

먼저 `README.md`, `docs/preliminary_work/README_control_pc.md`, `docs/preliminary_work/README_vision_pc.md`, `.gitignore`를 읽어라. 문서는 기존 스타일대로 짧고 실제 실행 가능한 내용만 기록한다.

## clone에 포함되는 것과 포함되지 않는 것

- `docs/preliminary_work/README_control_pc.md`: 제어 PC 터미널 1~7의 준비 절차와 하단의 비전 PC 작업 목록.
- `docs/preliminary_work/README_vision_pc.md`: 비전 PC의 실행 및 확인 절차.
- `src/control_pc/rohand_selector/`: 제어 PC의 손 선택용 wrapper. 비전 PC에서 실행할 프로그램이 아니다.
- `legacy/`, `build/`, `install/`, `log/`는 ignore되어 clone에 포함되지 않는다.
- FoundationPose, AnyGrasp, ZED 프로젝트와 가상환경은 이 저장소가 아니라 **비전 PC에 이미 있는 환경**을 찾아야 한다. Control PC의 `/home/rico/...` 경로가 여기에도 있다고 가정하지 않는다.

## 이번 비전 PC 작업

1. 기존 ZED2i / FoundationPose / Dual Pipe Tracking 프로젝트, 실행 스크립트, Python·CUDA·ROS2·container 환경을 읽기 전용으로 먼저 조사한다.
2. 현재 실행 중인 관련 프로세스와 ROS node/topic을 확인한다. 과거 코드, 현재 소스, 실제 실행 파일을 구분한다.
3. grasping 후 기본 자세에서 두 Pipe를 추적하는 실제 절차를 찾는다. CAD 선택, mask/초기 등록, tracking 재시작·lost 처리를 포함한다. 손에 가려진 상태에서 초기 등록이 가능한지도 확인한다.
4. 두 Pipe pose의 발행 인터페이스와 Control PC 수신 경로를 확인한다. 추가 adapter가 필요하면 기존 소스를 보존하고 이 저장소 안에 최소한으로 구성한다.
5. `docs/preliminary_work/README_vision_pc.md`에 터미널별 경로·환경 활성화·실행 명령·수신 확인 방법을 보완한다. 제어 PC 절차는 유지한다.

카메라·tracking 실행이 필요하면 먼저 해당 entry point에 로봇 구동 부작용이 없는지 확인한다. GUI의 전체 실행이나 로봇 이동 스크립트를 함께 시작하지 않는다.

## Control PC에서 확인한 인터페이스

2026-11-03 조사 기록이며, 실행 여부와 네트워크 주소는 연결 시 재확인한다.

| 항목 | 확인 내용 |
|---|---|
| ROS2 | Humble, `ROS_DOMAIN_ID=8`, `ROS_LOCALHOST_ONLY=0` |
| 실제 RMW | `rmw_fastrtps_cpp` 사용 관측. 비전 PC 설정은 미확인 |
| Control PC 주소 | 유선 `192.168.2.8/24`, Wi-Fi `192.168.10.104/24` |
| 왼팔 TCP feedback | `/left/robot/current/base_eef_pose`, `geometry_msgs/msg/PoseStamped` |
| 오른팔 TCP feedback | `/right/robot/current/base_eef_pose`, `geometry_msgs/msg/PoseStamped` |
| feedback 계약 | 위치 m, quaternion, 각각 `left_robot_base_frame` / `right_robot_base_frame`, 약 33.33 Hz |
| feedback 발행 주체 | Control PC의 양팔 `omega7_cartesian_slave`. 이 프로그램 종료 시 feedback도 중단됨 |

현재 teleop 경로는 Omega → `/omega7/live/{left,right}/state` (`Joy`) → native Cartesian slave → xArm SDK다. `/leftarm/Teleop_GT_Local`, `/rightarm/Teleop_GT_Local`은 당시 subscriber가 없는 호환 출력이었으며, Vision pose를 여기에 연결하면 안 된다. 현행 teleop에는 자율제어 전환 기능이 없다.

## Vision 출력에서 반드시 확인할 것

과거 2026-09-18 기록에는 아래 인터페이스가 있었다. **현재도 같다고 단정하지 말고 현지 소스와 메시지로 확인한다.**

- `/vision/left_pipe/pose`: `geometry_msgs/msg/PoseStamped`, 약 15 Hz, frame `camera_left_optical_frame`.
- `/vision/left_pipe/tracking_status`: `TRACKING` 수신 기록. 현재 message type과 상태값 정의 확인 필요.
- 오른쪽 Pipe의 정확한 topic/type은 미확인이다. 좌우 이름을 추측하지 않는다.

각 Pipe에 대해 다음을 정리한다.

- publisher/node, topic/type, QoS, 실제 주기, 두 Pipe ID의 안정적인 대응.
- 위치 단위, quaternion 순서, camera optical frame, CAD 원점·축·mesh 단위. 중심 보정된 object frame인지도 확인.
- timestamp가 이미지 취득 시각인지 추론 완료/발행 시각인지, 양 PC 시간 동기화와 지연.
- tracking lost 시 pose 중단/마지막 값 재발행 여부, 상태 topic과 stale 판단 방법. 오래된 pose에 새 timestamp만 붙여 유효한 것으로 보이게 하지 않는다.
- DDS discovery·네트워크·domain/RMW 설정과 Control PC에서의 실제 수신 여부. 로컬 수신만으로 PC 간 통신 성공이라고 보고하지 않는다.

## 좌표계와 미검증 사항

- S: TORSO 공통 base, L/R: 각 xArm base, C: ZED camera, E: 설정된 TCP, P: Pipe CAD frame.
- `A_T_B`는 B를 A에서 표현한 변환이며 `p_A = A_T_B * p_B`이다. Vision pose가 `C_T_P`인지 확인한다.
- 기존 `Convert_CamtoBase`의 `T_sc`는 `S_T_C`로 사용됐지만 현재 optical frame과의 일치 및 실측 검증은 미완료다.
- `L_T_C = inverse(S_T_L) * S_T_C`, 오른쪽도 동일한 관계다. 기존 후보 calibration을 검증값으로 취급하거나 임의로 새 값을 만들지 않는다.
- 과거 왼팔 Auto Align prototype은 `camera_color_optical_frame`을 기대했지만 Vision 기록은 `camera_left_optical_frame`이었다. 이름 변경만으로 같은 좌표계가 되지 않는다.
- 추후 Control PC에서는 현재/목표 Pipe pose로 목표 EEF를 계산하고 실제 TCP feedback으로 보정한다. 비전 PC에서 로봇 운동 명령을 생성·발행하는 단계까지 확대하지 않는다.

## 작업 제한과 결과

- 기존 비전 프로젝트·가상환경·container·시스템 설정은 보존한다. 재설치, container 재생성, `.bashrc` 변경, 기존 프로세스 임의 종료를 하지 않는다.
- 수정은 이 clone 안의 필요한 새 adapter·설정·문서로 한정한다. 기존 외부 소스가 꼭 바뀌어야 한다면 이유와 범위를 먼저 설명한다.
- 실제 로봇/손의 motion command와 제어 topic 발행은 하지 않는다. 인증정보·비밀번호·라이선스 키를 출력하지 않는다.
- Git 설정은 필요 시 이 저장소에만 `planta223 <planta223@naver.com>`을 사용한다. 전역 설정 변경과 임의 commit/push는 하지 않는다.
- 조사 결과는 **확인 사실 / 미확인**을 구분하고 실제 파일·함수·topic을 근거로 보고한다. 실행하지 않은 항목을 성공했다고 보고하지 않는다.
- 최종적으로 비전 PC의 실행 절차, 양쪽 pose 계약, Control PC 수신 검증 결과 또는 연결 미확인 이유, 다음으로 필요한 작업을 짧게 정리한다.
