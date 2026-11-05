# 정적 좌표계 검증

공식 `xarm_description`의 양팔 xArm7 모델에 `init 1` 목표 관절각을 적용해 RViz에 표시한다.
실제 로봇에 접속하거나 SDK·제어 노드를 실행하지 않는다. 실측 feedback 표시는 아직 없다.

- 공통 토르소 기준, 양팔 base·flange·TCP 축을 표시한다. 토르소·손 형상은 포함하지 않는다.
- `../../config/coordinate_frames.yaml`을 직접 읽으며 장착 변환과 TCP를 중복 저장하지 않는다.
- `config/init1.yaml`은 외부 `~/project/xarm_sdk/example/init.cc`의 SET1을 기록한 목표 관절각이다. degree를 rad로 변환해 사용한다.
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
ros2 launch simulation_validation init1.launch.py
```

RViz에서 실제 `init 1` 자세와 양팔 장착 방향·관절 배치·flange·TCP 축을 비교한다. 좌표축의 X/Y/Z는 빨강/초록/파랑이다.
전체 관절 좌표계와 카메라 축은 `All link frames (optional)` 표시를 켜서 확인할 수 있다.
기존 `T_sr`와 TCP offset의 실측 정확도, 실제 관절각과 SET1 목표값의 일치는 별도 확인이 필요하다.
표준 xArm7 기본 기구학을 사용하며 개체별 보정은 아직 적용하지 않는다.
1300 계열 도구 장착면 모델이 필요한 경우 해당 팔에 `model1300_left:=true` 또는 `model1300_right:=true`를 추가한다.

GUI 없이 관절각·TF만 확인하려면 같은 launch에 `rviz:=false`를 추가한다. 종료는 Ctrl+C다.
공통 설정 경로는 자동으로 찾으며, 다른 파일을 확인할 때는 `frames_file:=/절대경로/coordinate_frames.yaml`을 지정한다.
VS Code Snap의 GTK 경로는 RViz 프로세스에서만 제외하며 시스템 환경은 변경하지 않는다.

## 검사

빌드 환경을 불러온 상태에서 실행한다. 로봇은 사용하지 않는다.

```bash
python3 -m unittest discover \
  -s main/control_pc/src/simulation_validation/test -p 'test_*.py' -v
```
