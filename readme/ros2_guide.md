# ROS 2 실행 가이드

## 의존성

### Python

```bash
pip install -r requirements.txt
```

### ROS 2

- Ubuntu 24.04 + ROS 2 Jazzy

```bash
sudo apt install ros-jazzy-sensor-msgs ros-jazzy-std-msgs ros-jazzy-trajectory-msgs
```

---

## 설치

### Docker (권장)

호스트와 컨테이너 모두에서 DDS 전송을 UDPv4로 고정해야 통신이 안정적입니다.

```bash
# 0. 호스트에서 먼저 설정 (필수)
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# 1. 이미지 빌드
./docker/docker.sh build

# 2. 컨테이너 접속
./docker/docker.sh enter
```

### 로컬 빌드 (Ubuntu 24.04)

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules <repo_url>

cd ~/ros2_ws
pip install -r src/atlas_hand_ROS2/requirements.txt
colcon build --packages-select atlas_hand
source install/setup.bash
```

이미 클론한 경우 서브모듈을 별도로 초기화합니다.

```bash
git submodule update --init
```

---

## 실행

### 전체 시스템 시작

```bash
# 호스트에서도 반드시 설정
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# 기본 실행 (input_source=atlas, hand_type=both, robot_config=base_hand)
ros2 launch atlas_hand atlas_hand.launch.py

# 입력 소스 선택
ros2 launch atlas_hand atlas_hand.launch.py input_source:=atlas
ros2 launch atlas_hand atlas_hand.launch.py input_source:=meta_quest

# 옵션 조합 예시
# hand_type    : left | right | both
# robot_config : base_hand | orca_hand | robotis_hx5_d20 | allegro_hand | leap_hand | schunk_svh | tesollo_dg5f
# input_source : atlas | meta_quest
ros2 launch atlas_hand atlas_hand.launch.py hand_type:=right robot_config:=orca_hand input_source:=meta_quest
```

### 개별 노드 실행

```bash
# 입력 수신 노드 (기본: atlas)
ros2 run atlas_hand input_receiver
ros2 run atlas_hand input_receiver --ros-args -p input_source:=meta_quest

# 리타겟팅 노드
ros2 run atlas_hand retarget --ros-args -p hand_type:=left
ros2 run atlas_hand retarget --ros-args -p hand_type:=right -p robot_config:=orca_hand

# 3D 시각화 노드 (Rerun)
ros2 run atlas_hand visualizer left spawn    # 로컬 뷰어
ros2 run atlas_hand visualizer left connect  # 외부 뷰어 연결

# URDF 뷰어 (RViz2) — model: CONFIG_REGISTRY 키 중 선택  /  side: left | right
ros2 launch atlas_hand hand_view.launch.py
ros2 launch atlas_hand hand_view.launch.py model:=orca_hand       side:=right
ros2 launch atlas_hand hand_view.launch.py model:=robotis_hx5_d20 side:=left
```

### 토픽 확인

```bash
# 쿼터니언 수신 확인
ros2 topic echo /left_hand/quaternions
ros2 topic echo /right_hand/quaternions

# 연결 상태 확인
ros2 topic echo /left_hand/connected

# 관절 각도 출력 확인
ros2 topic echo /joint_states
```

### 햅틱 테스트 (Atlas 모드 전용)

```bash
python scripts/test_haptic_left.py

# ROS 2 CLI로 직접 발행
ros2 topic pub --once /left_hand/haptic std_msgs/msg/Int32MultiArray "data: [3, 3, 3, 3, 3]"
ros2 topic pub --once /left_hand/haptic/off std_msgs/msg/Empty "{}"
```

---

## 파라미터

### `input_receiver` 노드

| 파라미터       | 기본값  | 설명                   |
| -------------- | ------- | ---------------------- |
| `input_source` | `atlas` | `atlas` / `meta_quest` |

네트워크 설정(IP, 포트 등)은 [atlas_hand_core/config.py](../atlas_hand_core/config.py)에서 직접 수정합니다.

### `retarget` 노드

| 파라미터          | 기본값         | 설명                                                |
| ----------------- | -------------- | --------------------------------------------------- |
| `hand_type`       | `left`         | `left` / `right`                                    |
| `robot_config`    | `base_hand`    | `base_hand` / `orca_hand` / `robotis_hx5_d20` / `allegro_hand` / `leap_hand` / `schunk_svh` / `tesollo_dg5f` |
| `vector_weight`   | `1.0`          | Stage 1 시간 비중 (VectorOptimizer)                 |
| `position_weight` | `4.0`          | Stage 2 시간 비중 (PositionOptimizer)               |
| `tf_parent_frame` | *(wrist link)* | TF parent frame                                     |

### 코드 상수 ([atlas_hand_core/config.py](../atlas_hand_core/config.py))

| 상수                     | 기본값  | 설명                              |
| ------------------------ | ------- | --------------------------------- |
| `CONTROL_TIMER_SEC`      | `0.02`  | 제어 루프 주기 (50 Hz)            |
| `IK_MAX_TIME`            | `0.02`  | 두 단계 합산 IK 시간 예산 (20 ms) |
| `HUBER_DELTA`            | `0.025` | Huber loss δ                      |
| `NORM_DELTA`             | `0.01`  | Normal loss δ                     |
| `HEARTBEAT_INTERVAL_SEC` | `1.0`   | Atlas 하트비트 송신 간격          |

---

## 새 로봇 핸드 추가

상세 튜토리얼: [adding_hand_config.md](adding_hand_config.md)

1. `HandConfig`를 상속하는 클래스를 [atlas_hand_core/hand_configs.py](../atlas_hand_core/hand_configs.py)에 구현
2. `CONFIG_REGISTRY`에 키 등록
3. 실행 시 `robot_config` 인자로 선택

```bash
ros2 launch atlas_hand atlas_hand.launch.py robot_config:=<key>
ros2 run atlas_hand retarget --ros-args -p robot_config:=<key>
```
