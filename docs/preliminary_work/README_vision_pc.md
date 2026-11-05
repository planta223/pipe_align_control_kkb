# Auto Align 사전작업 — 비전 PC

[제어 PC 준비](README_control_pc.md)를 마치고 두 Pipe를 잡은 기본 자세에서 시작한다.
비전 PC에서 ZED2i 영상을 발행하고, YOLO로 초기 mask를 생성한 뒤 FoundationPose로 두 Pipe를 추적한다.
2026-11-05 로컬 소스 기준으로 좌우 pose 발행과 status 제거를 반영했다. 비전 실행은 사용자 정상 확인을 받았으며, Control PC의 실제 수신은 아직 확인 전이다.

## 실행 환경

- 기존 프로젝트: `~/Pipe_Align_kkb/`. 이 문서가 있는 `pipe_align_control_kkb` clone과 별개다.
- ZED2i publisher: `~/Pipe_Align_kkb/ZED2i_ROS/cam_zed.py`, 호스트 ROS2 Humble + conda `zed_cam`.
- 두 Pipe tracking: `~/Pipe_Align_kkb/MULTI_OBJECT_TRACKING/scripts/test_multi_object.py`.
- FoundationPose: `~/Pipe_Align_kkb/FoundationPose`, 기존 Docker container `foundationpose`의 ROS2 Foxy + `/opt/conda/envs/my/bin/python`.
- 호스트와 container 및 Control PC는 `ROS_DOMAIN_ID=8`, `ROS_LOCALHOST_ONLY=0`을 사용한다.

기존 container는 host network와 `/home` mount가 설정되어 있다.
아래에서는 기존 container만 시작한다. `FoundationPose/docker/run_container.sh`는 기존 container를 삭제하므로 사용하지 않는다.

## 1. 터미널 1 — ZED2i 영상 발행

ZED2i를 연결한 뒤 비전 PC 호스트에서 실행한다. 이 터미널은 켜 둔다.
`zed_cam` 환경에서 `pyzed.sl`, `rclpy`, `cv_bridge`, OpenCV import를 확인했다.

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate zed_cam
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
cd ~/Pipe_Align_kkb/ZED2i_ROS
python cam_zed.py
```

기본 설정은 컬러 1280×720, 깊이 640×360, 15 FPS, NEURAL depth다.
별도 카메라 GUI는 기본으로 열리지 않는다.

| Topic | Type |
|---|---|
| `/cam/color/compressed` | `sensor_msgs/msg/CompressedImage` |
| `/cam/depth/compressed` | `sensor_msgs/msg/CompressedImage` |
| `/cam/color/camera_info` | `sensor_msgs/msg/CameraInfo` |

## 2. 터미널 2 — FoundationPose 두 Pipe 추적

비전 PC의 그래픽 세션에서 새 터미널을 연다. tracking 창을 표시하므로 기존 root container에 X11 접근을 허용한다.

```bash
xhost +si:localuser:root
docker start foundationpose
docker exec -it -e DISPLAY="$DISPLAY" foundationpose bash
```

이후 명령은 **container 안에서** 실행한다. 호스트 Humble과 구분하여 Foxy 환경을 사용한다.
UDP profile은 기존 host/container 사이 DDS 통신용 설정이다.
아래 명령은 초기 영상에서 왼팔이 잡은 Pipe가 영상 왼쪽(`pipe1`)에 있는 경우다. 영상 오른쪽에 있다면 `--left-pipe-object-id pipe2`로 바꾼다.
지정한 ID가 left, 나머지 ID가 right로 발행된다. 양손이 각각 한 Pipe를 잡은 조건이며, 추적 중 영상 위치가 바뀌어도 이 대응은 유지된다.

```bash
cd /home/rico/Pipe_Align_kkb/MULTI_OBJECT_TRACKING
source /opt/ros/foxy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/ros2/fastdds_udp.xml"

/opt/conda/envs/my/bin/python scripts/test_multi_object.py \
  --foundationpose-root /home/rico/Pipe_Align_kkb/FoundationPose \
  --camera-type cam_ros_zed2i \
  --pipe1-model-path models/pipe1.obj \
  --pipe2-model-path models/pipe2.obj \
  --mesh-scale-to-meter 0.001 \
  --segmentation-mode yolo \
  --yolo-model-path models/yolo/pipe_seg.pt \
  --yolo-confidence 0.5 \
  --yolo-class-id 0 \
  --yolo-device cuda:0 \
  --register-refine-iter 5 \
  --track-refine-iter 2 \
  --z-axis-stabilization \
  --left-pipe-object-id pipe1
```

- `--show-auto-mask` : YOLO 초기 mask 확인 창을 띄운다.
- `--register-refine-iter 5`: 초기 등록 시 pose refinement를 5회 수행한다. 기본값은 5다.
- `--track-refine-iter 2`: 이후 프레임마다 tracking refinement를 2회 수행한다. 기본값은 2다.
- `--z-axis-stabilization`: 객체 Z축 방향의 연속성을 유지하고, Z축 주위 회전을 제거하여 X/Y축을 안정화한다. 위치는 유지하며 `--task-symmetry-output`과 동시에 사용할 수 없다.

### YOLO 초기 mask 확인

로컬의 `models/yolo/pipe_seg.pt`를 사용한다. YOLO는 같은 고정 영상에서 confidence가 높은 두 instance를 골라 영상 왼쪽부터 `pipe1`, `pipe2`로 배정한다.
mask 확인과 초기 등록이 끝날 때까지 Pipe 자세를 유지한다.
아래 확인 창은 실행 명령에 `--show-auto-mask`를 추가한 경우에만 열린다. 생략하면 YOLO 검출 후 바로 초기 등록을 진행한다.

1. `YOLO initial masks - press any key` 창에서 Pipe1(노란색), Pipe2(자홍색)의 mask가 각 Pipe를 덮는지 확인한다.
2. `--left-pipe-object-id`로 지정한 객체가 실제 왼팔이 잡은 Pipe인지 확인한다. 영상의 좌우는 팔의 좌우를 의미하지 않는다.
3. 맞으면 아무 키를 눌러 FoundationPose 초기 등록과 tracking을 진행한다. 틀리면 터미널에서 Ctrl+C로 종료하고 대응이나 검출 조건을 조정하여 다시 실행한다.

- YOLO는 초기 등록용 mask를 한 번 생성하며, 이후 프레임은 FoundationPose로 추적한다.
- `--show-auto-mask`는 현재 디렉터리에 `yolo_initial_masks.png`도 저장한다.
- tracking 창에서 `q`/Esc: tracking 종료. 카메라 publisher는 터미널 1에 남는다.
- CAD 단위는 mm이며 `0.001`로 m로 변환한다. 현재 `pipe1.obj`, `pipe2.obj`의 파일 내용은 동일하다. 실제 두 Pipe 형상과의 일치는 별도 확인이 필요하다.

## 3. 터미널 3 — 비전 PC 출력 확인

비전 PC 호스트의 새 터미널에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 topic list -t
ros2 topic echo --once /vision/left_pipe/pose \
  --qos-reliability best_effort
ros2 topic echo --once /vision/right_pipe/pose \
  --qos-reliability best_effort
```

현재 로컬 소스는 **두 Pipe를 추적하고 좌우 pose를 각각 발행한다.** 별도 tracking status 토픽은 없다.

| Topic | Type | 내용 |
|---|---|---|
| `/vision/left_pipe/pose` | `geometry_msgs/msg/PoseStamped` | 왼팔에 대응시킨 Pipe의 유효한 pose |
| `/vision/right_pipe/pose` | `geometry_msgs/msg/PoseStamped` | 오른팔에 대응시킨 Pipe의 유효한 pose |

- pose는 camera optical frame에서 표현한 Pipe CAD frame의 변환 `C_T_P`다. 위치는 m, 회전은 quaternion `x,y,z,w`, frame은 `camera_left_optical_frame`이다.
- 화면의 bounding box 중심 보정용 변환은 발행하지 않는다.
- 양쪽 pose QoS는 BEST_EFFORT / VOLATILE / depth 1이며, 동일 영상의 결과는 같은 timestamp와 frame을 사용한다.
- pose timestamp는 입력 영상의 stamp를 유지한다. 현재 카메라 publisher가 붙이는 시각은 영상 발행 시각이며, ZED 하드웨어 취득 시각은 아니다.

## 4. Control PC — 수신 확인

비전 PC의 카메라와 tracking을 유지한 상태에서 Control PC의 확인용 터미널에서 실행한다. 별도 bridge나 자체 subscriber 프로그램은 필요하지 않다.

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
ros2 topic info -v /vision/left_pipe/pose
ros2 topic info -v /vision/right_pipe/pose
ros2 topic echo --once /vision/left_pipe/pose \
  --qos-reliability best_effort
ros2 topic echo --once /vision/right_pipe/pose \
  --qos-reliability best_effort
```

비전 PC에서의 수신과 별개로 Control PC에서 실제 메시지를 받아야 PC 간 통신이 확인된다.
양쪽 토픽의 publisher를 확인하고, 두 메시지가 실제 도착하는지 확인한다. 이는 수신 확인이며 자율 정렬 제어 연결을 의미하지 않는다.

## Tracking 재시작과 남은 작업

- tracking 실패나 잘못된 pose는 해당 객체의 pose 발행을 중단한다. 이전 pose에 새 timestamp를 붙여 재발행하지 않는다.
- 제어 PC는 양쪽 pose의 timestamp와 수신 timeout으로 최신성을 판단하고, 한쪽이라도 갱신이 끊기면 정렬을 중단해야 한다. timestamp 비교를 위해 양 PC 시간 동기화가 필요하다.
- 자동 재등록과 tracking 중 재등록 키는 없다. 터미널 2의 tracking을 종료하고 같은 명령으로 다시 실행하여 YOLO 검출부터 진행한다. 카메라 publisher는 유지할 수 있다.
- 손에 가려진 상태의 초기 등록 성공 여부, 실제 추적 성능, 양 PC 수신은 실행 확인이 필요하다.
- camera-to-base calibration 검증과 로봇 자율제어 전환은 별도 작업이다.

작업을 마치면 터미널 1의 카메라를 Ctrl+C로 종료하고, X11 접근 허용은 호스트에서 해제한다.

```bash
xhost -si:localuser:root
```

소스 기준: `ZED2i_ROS/cam_zed.py`, `MULTI_OBJECT_TRACKING/scripts/test_multi_object.py`, `segmentation/yolo/yolo_segmenter.py`, `pose_logging/ros_pose_publisher.py`.
