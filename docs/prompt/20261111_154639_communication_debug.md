# 비전 PC ↔ 제어 PC 통신 디버깅 인수인계

작성: 2026-11-11 15:46:39 KST. 양 PC에서 같은 문서를 사용하고 마지막 결과 표를 공유한다.

## 목표와 작업 범위

현재 로봇은 `pre_align_ready` 자세이며, 비전 pose를 이용한 다음 단계 개발 전에 실제 수신을 확인하려 한다. **기존 공유기가 새 공유기로 교체됐다.** 과거 IP·서브넷·공유기 설정이 그대로라고 가정하지 않는다.

목표는 비전 PC의 FoundationPose가 발행하는 양쪽 pose가 제어 PC에 지속적으로 도착하지 않는 원인을 구분하는 것이다. 이번 작업에서 로봇·손을 움직이거나 pre_align을 다시 실행하지 않는다. 실행 중인 RoHand 드라이버와 grasp 상태를 유지한다.

우선 읽기 전용으로 조사한다. 네트워크·방화벽·공유기·DDS XML 변경이나 프로그램 재시작이 필요하면 원인 증거와 구체적인 변경안을 먼저 정리한다. 컨테이너를 재생성하거나 ROS 배포판·패키지를 교체하지 않는다. `FoundationPose/docker/run_container.sh`는 기존 컨테이너를 삭제할 수 있으므로 실행하지 않는다.

통신 시험용 발행은 이 문서의 `/communication_debug/*` 문자열 토픽만 사용한다. 로봇·손·실제 vision pose 토픽으로 시험 명령이나 가짜 pose를 보내지 않는다.

## 현재까지 확인한 사실

- 제어 PC에서 Domain 8·FastDDS로 양쪽 pose를 8초간 구독했으나 publisher 0개, 수신 0건이었다.
- 같은 조사에서 RoHand·selector 노드는 발견됐지만 외부 카메라 publisher는 발견되지 않았다. 이것만으로 비전 PC에서 발행이 멈췄다고 단정할 수는 없다.
- 제어 PC 조사 시점의 주소는 아래와 같다. DHCP 주소는 실행 시 다시 확인한다.

| 구분 | 조사 시점 주소 | 용도 |
|---|---|---|
| 제어 PC 유선 `enp86s0` | `192.168.2.8/24` | 로봇 연결망 |
| 제어 PC Wi-Fi `wlo1` | `10.102.101.110/24` | 비전 PC와 연결할 망인지 확인 필요 |
| 제어 PC 기본 게이트웨이 | `10.102.101.1`, `wlo1` | 새 Wi-Fi망 |
| 과거 인수인계의 제어 PC Wi-Fi | `192.168.10.104/24` | 과거 기록이며 현재 주소로 사용하지 않음 |
| 비전 PC 주소·게이트웨이 | 미확인 | 현지에서 확인 |

로봇망 `192.168.2.0/24`와 비전 통신망을 구분한다. 비전 통신을 맞추기 위해 로봇 IP나 제어 PC의 로봇용 유선 주소를 바꾸지 않는다. Tailscale 등 추가 인터페이스가 있으므로 DDS가 어느 인터페이스를 사용하는지도 확인한다.

참고: [비전 PC 준비](../preliminary_work/README_vision_pc.md), [제어 PC 준비](../preliminary_work/README_control_pc.md), [현재 실행 방법](../control_pc/README_only_control.md).

## 확인할 인터페이스

| 항목 | 계약 |
|---|---|
| 왼쪽 pose | `/vision/left_pipe/pose`, `geometry_msgs/msg/PoseStamped` |
| 오른쪽 pose | `/vision/right_pipe/pose`, `geometry_msgs/msg/PoseStamped` |
| 좌표·단위 | `camera_left_optical_frame`, 위치 m, quaternion `x,y,z,w` |
| QoS | BEST_EFFORT / VOLATILE / depth 1 |
| timestamp | 입력 영상의 stamp 유지. 좌우가 같은 영상이면 같은 stamp |
| tracking 실패 | 해당 객체 pose 발행 중단. 별도 tracking status 토픽 없음 |
| 카메라 입력 | `/cam/color/compressed`, `/cam/depth/compressed`, `/cam/color/camera_info` |

비전 호스트는 ROS2 Humble, 기존 `foundationpose` 컨테이너는 ROS2 Foxy다. 기존 문서상 컨테이너는 host network이며, 실제 설정은 `docker inspect`로 재확인한다. 포트가 열리거나 토픽 이름이 보이는 것만으로 실제 pose 수신 성공이라고 보고하지 않는다.

## 1. 양 PC 호스트 — 현재 네트워크와 ROS 환경 기록

각 PC의 새 터미널에서 아래를 실행하고 PC 이름과 함께 결과를 기록한다.

```bash
hostname
date --iso-8601=seconds
ip -brief address
ip route
nmcli -f NAME,TYPE,DEVICE connection show --active
printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER RMW_FASTRTPS_USE_QOS_FROM_XML
```

`nmcli`가 없으면 IP·라우팅 결과와 실제 접속한 SSID를 기록한다. `printenv`의 출력이 없으면 해당 변수가 설정되지 않은 것이다. 새 터미널의 환경과 이미 실행 중인 카메라·tracker의 환경은 별개다. 새 창에서 `export`해도 기존 프로세스 설정은 바뀌지 않는다.

이후 **각 호스트의 모든 확인용 새 터미널**에서 다음을 실행한다. 컨테이너에는 Humble을 source하지 않는다.

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
```

기존 XML·Discovery Server 환경변수는 지우지 말고 먼저 기록한다. `--no-daemon`으로 CLI의 과거 discovery 캐시 영향을 줄인다.

새 공유기와 관련해 다음을 양 PC에서 대조한다.

- 실제 IP·프리픽스·게이트웨이가 무엇인지, 두 PC 사이에 라우팅이 되는지.
- 한쪽이 이전 공유기나 다른 SSID, 게스트 SSID, 다른 VLAN에 연결된 것은 아닌지. 같은 SSID도 단말 간 통신을 보장하지 않는다.
- AP/client isolation, 게스트망 단말 격리, 유선↔무선 격리 설정이 있는지.
- 멀티캐스트 차단·무선 멀티캐스트 필터·IGMP 관련 정책이 있는지. 설정 이름만 보고 임의로 일괄 변경하지 않는다.
- DDS 설정에 과거 `192.168.10.*` 주소나 이전 NIC 이름·Discovery Server 주소가 고정돼 있는지.

상대 PC의 **현재 통신용 IP**를 확인한 뒤 아래 변수의 설명 문자열을 실제 IP로 교체한다. 로봇 IP를 넣지 않는다.

제어 PC 호스트:

```bash
VISION_WIFI_IP='비전_PC의_실제_통신용_IP'
ip route get "$VISION_WIFI_IP"
ping -c 4 -W 1 "$VISION_WIFI_IP"
```

비전 PC 호스트:

```bash
CONTROL_WIFI_IP='제어_PC의_실제_통신용_IP'
ip route get "$CONTROL_WIFI_IP"
ping -c 4 -W 1 "$CONTROL_WIFI_IP"
```

`ip route get`의 `dev`·`src`가 의도한 PC 간 통신 인터페이스와 주소인지 확인한다. ping 실패는 ICMP 차단일 수도 있고, 성공해도 DDS UDP 통신 성공을 뜻하지 않는다.

## 2. 비전 PC 담당 — 발행 경로를 안쪽부터 확인

### 2-1. 호스트의 카메라와 tracker 상태

기존 실행 터미널의 오류·마지막 로그와 tracking 창 상태를 확인한다. 기존 프로그램을 중복 실행하지 않는다.

```bash
pgrep -af 'cam_zed.py|test_multi_object.py'
docker ps -a --filter name=foundationpose
docker inspect --format '{{.State.Status}} {{.HostConfig.NetworkMode}}' foundationpose
ros2 topic info --no-daemon --spin-time 3 -v /cam/color/compressed
ros2 topic info --no-daemon --spin-time 3 -v /cam/depth/compressed
ros2 topic info --no-daemon --spin-time 3 -v /cam/color/camera_info
timeout 10s ros2 topic echo --once /cam/color/camera_info sensor_msgs/msg/CameraInfo --qos-reliability best_effort
```

카메라 publisher 수뿐 아니라 tracker가 새 컬러·깊이 영상을 받는지도 기존 로그로 확인한다. 카메라 입력 기본값은 15 FPS지만, pose 출력도 같은 주기라고 단정하지 않는다.

tracker는 컨테이너 안에서 실행되므로 호스트의 `pgrep` 표시만으로 판단하지 않고 다음 단계에서도 확인한다. 카메라 입력이 멈췄다면 pose 통신보다 그 지점을 먼저 조사한다.

### 2-2. 컨테이너 안에서 pose 발행 확인

`foundationpose`가 이미 running인 경우 별도 터미널로 들어간다. 멈춰 있다면 그 사실을 기록하고 기존 [시작 절차](../preliminary_work/README_vision_pc.md)와 실제 종료 원인을 확인한다.

```bash
docker exec -it foundationpose bash
```

이하는 **컨테이너 안**에서 실행한다.

```bash
source /opt/ros/foxy/setup.bash
printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION FASTRTPS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER
pgrep -af test_multi_object.py
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=8
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/rico/Pipe_Align_kkb/MULTI_OBJECT_TRACKING/ros2/fastdds_udp.xml
ls -l "$FASTRTPS_DEFAULT_PROFILES_FILE"
ros2 topic info -v /vision/left_pipe/pose
ros2 topic info -v /vision/right_pipe/pose
timeout 10s ros2 topic echo --once /vision/left_pipe/pose geometry_msgs/msg/PoseStamped --qos-reliability best_effort
timeout 10s ros2 topic echo --once /vision/right_pipe/pose geometry_msgs/msg/PoseStamped --qos-reliability best_effort
```

기존 경로는 현지에서 확인한다. XML이 없다면 대체 설정을 만들지 말고 그 사실과 실제 tracker 시작 명령을 기록한다. 위 `export`는 확인용 CLI에 적용되며 실행 중인 tracker의 설정을 바꾸지 않는다.

- publisher 0: tracker 미실행·종료·ROS 발행 초기화 전·다른 Domain/설정을 확인한다.
- publisher는 있지만 pose가 오지 않음: 카메라 입력, YOLO mask 확인 대기, 초기 등록 미완료, tracking lost, 발행 조건·예외 로그를 확인한다.
- 한쪽만 수신: 해당 물체의 검출·등록·추적 상태와 left/right ID 대응을 확인한다.
- 컨테이너 안에서는 양쪽 모두 수신: 다음으로 호스트에서도 같은 결과인지 확인한다.

GUI의 `YOLO initial masks - press any key`에서 멈췄다면 초기 등록 대기 상태다. mask와 실물의 대응을 확인한 뒤 정상 절차에 따라 조작한다. tracking 실패 시 pose 발행을 멈추는 동작을 DDS 고장과 혼동하지 않는다.

### 2-3. 비전 PC 호스트에서 같은 pose 확인

호스트의 새 터미널에서 1절의 Humble 환경을 설정한 뒤 실행한다.

```bash
ros2 topic info --no-daemon --spin-time 3 -v /vision/left_pipe/pose
ros2 topic info --no-daemon --spin-time 3 -v /vision/right_pipe/pose
timeout 10s ros2 topic echo --once /vision/left_pipe/pose geometry_msgs/msg/PoseStamped --qos-reliability best_effort
timeout 10s ros2 topic echo --once /vision/right_pipe/pose geometry_msgs/msg/PoseStamped --qos-reliability best_effort
```

컨테이너 안에서는 성공하고 호스트에서 실패한다면, 공유기보다 먼저 컨테이너의 network mode, Domain, XML transport, 공유 메모리/UDP 설정을 조사한다. host network여도 실제 수신 검증을 생략하지 않는다.

## 3. 제어 PC 담당 — 외부 pose 수신 확인

pre_align 이동을 재실행하지 않는다. 호스트의 새 터미널에서 1절의 Humble 환경을 설정하고 아래를 실행한다.

```bash
ros2 topic list --no-daemon --spin-time 3 -t
ros2 topic info --no-daemon --spin-time 3 -v /vision/left_pipe/pose
ros2 topic info --no-daemon --spin-time 3 -v /vision/right_pipe/pose
timeout 10s ros2 topic echo --once /vision/left_pipe/pose geometry_msgs/msg/PoseStamped --qos-reliability best_effort
timeout 10s ros2 topic echo --once /vision/right_pipe/pose geometry_msgs/msg/PoseStamped --qos-reliability best_effort
```

`Publisher count`와 node/type/QoS를 기록한다. 구독 프로그램이 토픽을 생성해 이름이 보일 수도 있으므로, 목록에 이름이 있는 것만으로 publisher 존재를 판단하지 않는다. 메시지를 받으면 `frame_id`와 stamp를 확인한다.

지속 갱신은 아래 명령을 하나씩 실행해 확인한다.

```bash
timeout 15s ros2 topic echo /vision/left_pipe/pose geometry_msgs/msg/PoseStamped --field header --qos-reliability best_effort
timeout 15s ros2 topic echo /vision/right_pipe/pose geometry_msgs/msg/PoseStamped --field header --qos-reliability best_effort
```

`--once`가 출력 없이 10초 뒤 끝나면 수신 미확인이다. 실행 직후 `echo $?`로 확인하면 timeout의 기본 종료 코드는 124다. 지속 echo는 15초 후 timeout되는 것이 정상이며, 그 전 메시지와 stamp가 반복 갱신됐는지를 본다.

## 4. 양 PC 담당 — 새 공유기 구간과 DDS 분리 시험

비전 호스트에서는 pose가 오지만 제어 PC에는 오지 않을 때 진행한다. 양 PC 호스트에서 1절 환경을 설정하고, 수신 측을 먼저 켠 뒤 다른 PC에서 송신한다.

### 4-1. 멀티캐스트 양방향 확인

제어 PC 호스트에서 대기:

```bash
timeout 15s ros2 multicast receive
```

대기 중 비전 PC 호스트에서 송신:

```bash
ros2 multicast send
```

그다음 방향을 바꿔 비전 PC에서 `receive`, 제어 PC에서 `send`를 실행한다. 각 수신 터미널의 `Hello World!` 수신 여부를 기록한다.

이 시험은 기본 멀티캐스트 경로만 확인한다. 다중 NIC 환경에서는 의도와 다른 인터페이스를 사용할 수 있다. `ip route get 225.0.0.1`도 기록하고, 필요하면 실제 패킷 인터페이스를 확인한다. 성공해도 DDS 전체 통신을 보증하지 않고, 실패해도 공유기만을 원인으로 단정하지 않는다.

### 4-2. 카메라와 무관한 DDS 문자열 시험

제어 PC 호스트에서 먼저 대기:

```bash
timeout 25s ros2 topic echo /communication_debug/vision_to_control std_msgs/msg/String --qos-reliability best_effort --qos-durability volatile
```

대기 중 비전 PC 호스트에서 발행:

```bash
timeout 20s ros2 topic pub -r 1 /communication_debug/vision_to_control std_msgs/msg/String "{data: 'vision-to-control'}" --qos-reliability best_effort --qos-durability volatile
```

반대 방향은 비전 PC 호스트에서 먼저 대기:

```bash
timeout 25s ros2 topic echo /communication_debug/control_to_vision std_msgs/msg/String --qos-reliability best_effort --qos-durability volatile
```

제어 PC 호스트에서 발행:

```bash
timeout 20s ros2 topic pub -r 1 /communication_debug/control_to_vision std_msgs/msg/String "{data: 'control-to-vision'}" --qos-reliability best_effort --qos-durability volatile
```

문자열은 양방향으로 도착하지만 실제 pose만 안 오면 tracker 프로세스의 DDS 설정·발행 조건·QoS로 범위를 좁힌다. 호스트끼리 성공해도 컨테이너 발행 경로는 별도로 확인해야 한다. 문자열도 안 오면 PC 간 discovery/UDP/인터페이스/공유기 정책을 조사한다.

### 4-3. DDS XML·방화벽·패킷 확인

각 PC 및 컨테이너의 실제 XML 경로를 확인한다. 설정을 덮어쓰지 않고 다음 항목을 읽는다.

- `interfaceWhiteList`, `initialPeersList`, locator/address에 이전 IP가 고정되어 있는지.
- `ROS_DISCOVERY_SERVER`가 이전 장비 주소를 가리키는지.
- UDP transport를 사용하는지, 실제 LAN에서 도달할 수 없는 로봇용 유선 IP·VPN 주소를 광고하는지.
- 공유기의 AP/client isolation, 게스트망/VLAN 분리, 유무선 격리, 멀티캐스트 필터 정책.

방화벽은 `ufw status verbose` 또는 해당 PC가 실제 사용하는 방화벽 규칙을 읽어 확인한다. 권한이 필요하면 현지 담당자가 확인한다. 방화벽을 통째로 끄거나 모든 UDP 포트를 여는 조치는 하지 않는다.

원인이 남으면 실제 통신용 NIC에서 `tcpdump`로 discovery/UDP 송수신을 확인한다. 예를 들어 `tcpdump -ni 실제_NIC 'udp and host 상대_PC_IP'` 형태로 범위를 한정한다. `ros2 multicast` 시험은 멀티캐스트 목적지라 이 필터로 송신이 빠질 수 있으므로 별도 필터가 필요하다. 송신 NIC에 패킷이 나가는지, 상대 수신 NIC까지 오는지를 양쪽에서 비교해 차단 구간을 찾는다. Domain 8의 모든 DDS 트래픽을 단일 고정 포트 하나로 간주하지 않는다.

## 5. 시간 동기화와 좌표계는 통신 이후 확인

양 PC에서 다음을 기록한다.

```bash
date --iso-8601=ns
timedatectl show -p NTPSynchronized -p TimeUSec
```

단순히 두 명령의 출력 시각 차이를 정밀한 clock offset으로 간주하지 않는다. 실제 사용 중인 NTP/chrony 상태와 필요 시 별도 측정을 확인한다. PC 시간 오차는 pose의 stale 판정을 망가뜨릴 수 있으나 publisher 0개의 직접 증거는 아니다.

`fixed_frames.yaml`의 카메라 변환이 `null`인 것은 다음 로봇 제어 단계의 별도 과제다. pose 토픽의 발견·수신 실패를 이 값으로 설명하지 않는다.

## 6. 결과 해석과 다음 조치

| 결과 | 우선 조사할 구간 |
|---|---|
| 컨테이너에서도 publisher 0 | tracker 실행·초기화·Domain/실행 환경 |
| publisher는 있으나 컨테이너에서도 pose 없음 | 카메라 입력·mask/등록 대기·tracking 실패·발행 조건 |
| 컨테이너 성공, 비전 호스트 실패 | 컨테이너 network/transport/XML/ROS 환경 |
| 비전 호스트 성공, 제어 PC 실패 | PC 간 DDS·라우팅·공유기 격리·멀티캐스트·방화벽 |
| 멀티캐스트 실패 | NIC 선택·LAN/VLAN·공유기 필터·호스트 방화벽 조사. 원인 단정 금지 |
| 문자열 성공, 실제 pose 실패 | tracker의 transport/광고 주소·QoS·실제 발행 지속 여부 |
| 한쪽 pose만 수신 | 해당 객체 tracking과 좌우 매핑·그 토픽의 publisher 확인 |
| 두 pose 수신, stamp 정지/오래됨 | 카메라 갱신·입력 지연·시간 동기화·stale 판정 |

**성공 기준:** 제어 PC에서 양쪽 실제 pose가 반복 수신되고, frame과 stamp가 정상이며 새 영상에 따라 stamp가 갱신돼야 한다. timestamp 최신성은 양 PC 시계 상태를 함께 확인한다. 통신 성공은 정렬 제어 성공을 뜻하지 않는다.

## 7. 양 PC가 공유할 기록

| 항목 | 비전 PC | 제어 PC |
|---|---|---|
| 확인 시각·호스트명 | | |
| 통신 NIC·현재 IP/프리픽스 | | |
| SSID·게이트웨이·상대 IP로 가는 dev/src | | |
| 상대 PC ping | | |
| 확인용 CLI의 Domain/RMW/localhost | | |
| 실제 publisher 프로세스의 환경/XML | | 해당 없음 |
| 컨테이너 network mode | | 해당 없음 |
| 로컬 카메라 입력 상태 | | 해당 없음 |
| 컨테이너 안 좌우 publisher 수·실제 수신 | | 해당 없음 |
| 호스트 좌우 publisher 수·실제 수신 | | |
| 좌우 frame·stamp 갱신·끊김 | | |
| 멀티캐스트 송신/수신 | | |
| 시험 문자열 송신/수신 | | |
| 시간 동기화 상태 | | |
| 최초 실패 지점·관련 로그 | | |

기록에는 실행한 명령과 실제 출력을 붙인다. 확인된 사실, 추정 원인, 아직 미확인인 항목을 구분한다. 변경 제안이 생기면 대상 PC·정확한 설정 항목·변경 전후 값·되돌리는 방법을 함께 제시한다. 원인이 확인되기 전에는 기존 비전/제어 코드나 로봇 설정을 수정하지 않는다.
