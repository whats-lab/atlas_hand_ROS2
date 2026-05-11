# atlas_hand

AGA 글러브 데이터를 수신하여 로봇 손을 실시간으로 제어하는 패키지입니다.  
**ROS 2 모드**와 **Standalone 모드** 두 가지 방식으로 실행할 수 있습니다.

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-Proprietary-red.svg)](LICENSE)

![robotis_hx5_d20|Real_hand|body_part_hand](readme/real2sim_hand.jpg)

## 개요

AGA 글러브의 17개 관절 쿼터니언을 OSC 프로토콜로 수신하여  
Pinocchio 기반 Forward Kinematics → dex-retargeting IK 파이프라인으로 로봇 핸드의 관절 각도로 변환합니다.

모든 연산 및 시각화의 좌표축은 **Unity 좌표계(Left-Handed)** 를 기준으로 설계되었습니다.

| 모드 | 설명 | 필요 환경 |
| ---- | ---- | --------- |
| **ROS 2** | 전체 노드 파이프라인, 햅틱 피드백, RViz2 시각화 | Ubuntu 24.04 + ROS 2 Jazzy |
| **Standalone** | ROS 없이 OSC → FK/IK → Rerun 시각화 또는 콘솔 출력 | Python 3.10+ |

---

## 시스템 아키텍처

### ROS 2 모드

```
[Air Glove Atlas 디바이스]
  OSC 수신 (/left/quat/get, /right/quat/get)
  OSC 수신 (/device/status/get, /heartbeat/get)
  OSC 수신 (/left/hapt/ret/get, /right/hapt/ret/get)  ← 햅틱 ACK
  OSC 수신 (/device/alarm/get)
        │ ▲
        │ │  OSC 송신 (/left/hapt/set, /right/hapt/set)  ← 햅틱 명령
        │ │  OSC 송신 (/device/status/get "4")            ← 하트비트
        ▼ │
┌──────────────────────────────────────────┐
│          osc_receiver_node               │
│  - 쿼터니언 수신 → ROS 2 발행             │
│  - 연결 상태 추적 → Bool 발행             │
│  - 햅틱 토픽 구독 → OSC 전송             │
└──────────────────────────────────────────┘
        │                   ▲
        │   ROS 2 Topics    │   ROS 2 Topics
        │  (quaternions,    │   (haptic 명령)
        │   connected)      │
        ▼               [다른 노드]
┌────────────────────────────────────────────────────────────────┐
│                     retargeting_node                            │
│  1. FK        — Pinocchio spherical joint → 23 joint positions │
│  2. 좌표 변환 — AGA frame → 로봇 frame (3×3 행렬)               │
│  3. 스케일 보정 — float 또는 손가락별 List[float]                │
│  4. Stage 1   — VectorOptimizer (손 방향/형태 정규화)            │
│  5. Stage 2   — PositionOptimizer (손끝 위치 정밀 보정)          │
│  6. 손목 직접 매핑 — swing-twist 분해 (설정 시)                  │
└────────────────────────────────────────────────────────────────┘
        │
        │   /joint_states          (sensor_msgs/JointState)
        │   /{side}_hand/wrist_xyz (std_msgs/Float32MultiArray, 디버그)
        ▼
  [Robot Hand]
```

> ROS 2 토픽 상세: [readme/topics.md](readme/topics.md)

### Standalone 모드

```
[Air Glove Atlas 디바이스]
        │ OSC UDP
        ▼
┌──────────────────────────────────────────┐
│           AGAOscHandler                  │
│  (atlas_hand_core/osc_handler.py)        │
│  - 쿼터니언 수신 → on_*_quat 콜백        │
│  - 연결 상태 추적 → on_status_change     │
│  - 햅틱 OSC 송신                         │
└──────────────────────────────────────────┘
        │ on_left/right_quat(17×4 array)
        ▼
┌──────────────────────────────────────────┐
│        standalone/visualize.py           │  ← Rerun 3D 시각화
│        standalone/retarget.py            │  ← FK → IK → 콘솔 출력
└──────────────────────────────────────────┘
```

---

## 패키지 구조

```
atlas_hand_ROS2/
├── package.xml
├── setup.py
├── setup.cfg
├── requirements.txt
│
├── atlas_hand/                           # ROS 2 노드 패키지
│   ├── config.py                         # ROS 2 토픽 이름 상수
│   ├── osc_receiver_node.py              # OSC 수신/송신 노드 (AGAOscHandler 래퍼)
│   ├── retargeting_node.py               # 리타겟팅 노드 (HandRetargeter 래퍼)
│   └── visualizer_node.py               # Rerun 3D 시각화 노드
│
├── atlas_hand_core/                      # 순수 Python 코어 (ROS 2 비의존)
│   ├── config.py                         # 하드웨어 / 알고리즘 상수
│   ├── osc_handler.py                    # AGAOscHandler — OSC 송수신 + 하트비트
│   ├── hand_configs.py                   # 핸드 모델별 리타겟팅 설정
│   ├── hand_spherical_fk.py              # Pinocchio FK + Rerun 시각화 클래스
│   └── retargeter.py                     # HandRetargeter — FK → IK 파이프라인
│
├── standalone/                           # ROS 2 없이 실행 가능한 스크립트
│   ├── visualize.py                      # OSC → FK → Rerun 시각화
│   └── retarget.py                       # OSC → FK → IK → 콘솔 출력
│
├── scripts/                              # 개발/테스트 유틸리티
│   └── test_haptic_left.py               # 왼손 햅틱 수동 테스트 (1/2 키)
│
├── models/                               # 핸드 모델 (URDF + 메쉬 + RViz 설정)
│   ├── base/                             # BodyParts3D 기반 기본 핸드
│   │   ├── urdf/                         # left.urdf / right.urdf
│   │   ├── meshes/                       # 시각화용 STL (left / right)
│   │   ├── assets/                       # 전체 해상도 메쉬
│   │   └── rviz/                         # base_left.rviz / base_right.rviz
│   ├── orca/                             # OrcaHand v2 (ETH Zurich)
│   │   ├── urdf/                         # left.urdf / right.urdf
│   │   ├── meshes/                       # STL (left / right)
│   │   └── rviz/                         # orca.rviz
│   └── robotis/                          # Robotis HX5 D20
│       ├── urdf/                         # hx5_d20_left.urdf / hx5_d20_right.urdf
│       └── rviz/                         # robotis.rviz
├── launch/
│   ├── atlas_hand.launch.py              # 메인 런처 (OSC + 리타겟팅)
│   └── hand_view.launch.py               # URDF 뷰어 (model × side 인자)
└── docker/
```

---

## 의존성

### Python (공통)

```bash
pip install -r requirements.txt
```

| 패키지            | 용도                        |
| ----------------- | --------------------------- |
| `numpy`           | 수치 연산                   |
| `scipy`           | 회전 변환                   |
| `pin`             | Pinocchio (FK)              |
| `dex-retargeting` | IK 최적화                   |
| `rerun-sdk`       | 3D 시각화 (standalone 전용) |
| `python-osc`      | OSC UDP 통신                |

### ROS 2 (ROS 2 모드 전용)

- Ubuntu 24.04 + ROS 2 Jazzy

```bash
sudo apt install ros-jazzy-sensor-msgs ros-jazzy-std-msgs ros-jazzy-trajectory-msgs
```

---

## 설치

### Standalone 모드

ROS 2 불필요 — Python 의존성만 설치하면 바로 실행 가능합니다.

```bash
git clone <repo_url>
cd atlas_hand_ROS2
pip install -r requirements.txt
```

### ROS 2 모드

의존성 패키지 설치가 복잡하므로 Docker 사용을 권장합니다.

#### Docker (권장)

호스트와 컨테이너 모두에서 DDS 전송을 UDPv4로 고정해야 통신이 안정적입니다.

```bash
# 0. 호스트에서 먼저 설정 (필수)
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# 1. 이미지 빌드
./docker/docker.sh build

# 2. 컨테이너 접속
./docker/docker.sh enter
```

#### 로컬 빌드 (Ubuntu 24.04)

```bash
cd ~/ros2_ws/src
git clone <repo_url>

cd ~/ros2_ws
pip install -r src/atlas_hand_ROS2/requirements.txt
colcon build --packages-select atlas_hand
source install/setup.bash
```

---

## 실행

### Standalone 모드

ROS 2 없이 글러브와 바로 연결하여 시각화하거나 IK 출력을 확인할 수 있습니다.

```bash
# Rerun 3D 시각화 (로컬 뷰어 실행)
python standalone/visualize.py left spawn
python standalone/visualize.py right spawn

# 외부 Rerun 뷰어에 연결
python standalone/visualize.py left connect

# FK → IK 리타겟팅 결과 콘솔 출력
python standalone/retarget.py left
python standalone/retarget.py right

# 특정 로봇 핸드 설정 지정
python standalone/retarget.py left orca_hand
python standalone/retarget.py right robotis_hx5
```

---

### ROS 2 모드

#### 전체 시스템 시작

```bash
# 호스트에서도 반드시 설정
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# 기본 실행 (hand_type=both, robot_config=base)
ros2 launch atlas_hand atlas_hand.launch.py

# 옵션 지정
# hand_type    : left | right | both
# robot_config : base | robotis_hx5 | orca_hand
ros2 launch atlas_hand atlas_hand.launch.py hand_type:=right robot_config:=orca_hand
```

#### 개별 노드 실행

```bash
# OSC 수신/송신 노드
ros2 run atlas_hand osc_receiver

# 리타겟팅 노드
ros2 run atlas_hand retarget --ros-args -p hand_type:=left
ros2 run atlas_hand retarget --ros-args -p hand_type:=right -p robot_config:=orca_hand

# 3D 시각화 노드 (Rerun)
ros2 run atlas_hand visualizer left spawn    # 로컬 뷰어
ros2 run atlas_hand visualizer left connect  # 외부 뷰어 연결

# URDF 뷰어 (RViz2)
# model: base | orca | robotis  /  side: left | right
ros2 launch atlas_hand hand_view.launch.py
ros2 launch atlas_hand hand_view.launch.py model:=orca    side:=right
ros2 launch atlas_hand hand_view.launch.py model:=robotis side:=left
```

#### 토픽 확인

```bash
# 쿼터니언 수신 확인
ros2 topic echo /left_hand/quaternions
ros2 topic echo /right_hand/quaternions

# 연결 상태 확인
ros2 topic echo /left_hand/connected

# 관절 각도 출력 확인
ros2 topic echo /joint_states

# 손목 디버그 (orca_hand 등 _WRIST_JOINTS 설정 시)
# data: [raw_x, raw_y, raw_z, raw_w, euler_x, euler_y, euler_z, swing_0, ...]
ros2 topic echo /right_hand/wrist_xyz
```

#### 햅틱 테스트

```bash
# 전용 테스트 스크립트 (1 → ON, 2 → OFF, q → 종료)
python scripts/test_haptic_left.py

# ROS 2 CLI로 직접 발행
ros2 topic pub --once /left_hand/haptic std_msgs/msg/Int32MultiArray "data: [100, 100, 100, 100, 100]"
ros2 topic pub --once /left_hand/haptic/off std_msgs/msg/Empty "{}"
```

---

## 주요 파라미터

### ROS 2 파라미터 (`--ros-args -p <param>:=<value>`)

| 파라미터          | 기본값         | 설명                                                |
| ----------------- | -------------- | --------------------------------------------------- |
| `hand_type`       | `left`         | `left` / `right`                                    |
| `robot_config`    | `base`         | 로봇 설정 키 (`base` / `robotis_hx5` / `orca_hand`) |
| `vector_weight`   | `1.0`          | Stage 1 시간 비중 (VectorOptimizer)                 |
| `position_weight` | `4.0`          | Stage 2 시간 비중 (PositionOptimizer)               |
| `tf_parent_frame` | *(wrist link)* | TF parent frame (기본값: wrist 링크명)              |
| `listen_ip`       | `0.0.0.0`      | OSC 수신 IP (osc_receiver)                          |
| `server_port`     | `4040`         | OSC 수신 포트                                       |
| `target_ip`       | `127.0.0.1`    | AGA 글러브 디바이스 IP                              |
| `client_port`     | `4042`         | 하트비트 송신 포트                                  |

### 코드 상수 ([atlas_hand_core/config.py](atlas_hand_core/config.py))

| 상수                | 기본값  | 설명                              |
| ------------------- | ------- | --------------------------------- |
| `CONTROL_TIMER_SEC` | `0.02`  | 제어 루프 주기 (50 Hz)            |
| `IK_MAX_TIME`       | `0.02`  | 두 단계 합산 IK 시간 예산 (20 ms) |
| `HUBER_DELTA`       | `0.025` | Huber loss δ                      |
| `NORM_DELTA`        | `0.01`  | Normal loss δ                     |
| `HEARTBEAT_INTERVAL_SEC` | `1.0` | 하트비트 송신 간격           |

---

## 새 로봇 핸드 추가

상세 튜토리얼: [readme/adding_hand_config.md](readme/adding_hand_config.md)

1. `HandConfig`를 상속하는 클래스를 [atlas_hand_core/hand_configs.py](atlas_hand_core/hand_configs.py)에 구현 (클래스 변수 선언만으로 완성)
2. `CONFIG_REGISTRY`에 키 등록
3. 실행 시 `robot_config` 인자로 선택

```bash
# ROS 2
ros2 run atlas_hand retarget --ros-args -p robot_config:=<key>

# Standalone
python standalone/retarget.py left <key>
```

---

## 라이선스

Proprietary — © WHATsLAB. All rights reserved.

본 소프트웨어의 소스 코드 및 알고리즘에 대한 권리는 WHATsLAB에 있으며, 무단 복제 및 배포를 금합니다.

---

### Third-Party Data & Attribution

본 프로젝트는 시각화 및 키네마틱스 모델링을 위해 아래의 외부 데이터를 포함하고 있으며, 각 데이터는 원저작자의 라이선스 정책을 따릅니다.

#### 1. ROBOTIS Hand 2 (URDF & Meshes)

Origin: [ROBOTIS-GIT/robotis_hand_2](https://github.com/ROBOTIS-GIT/robotis_hand)

Copyright: © ROBOTIS Co., Ltd.

License: Apache License 2.0

Location: [models/robotis/](models/robotis/)

Changes: 프로젝트의 ROS 2 환경에 맞춰 URDF 파일의 경로 수정 및 물리 파라미터 최적화가 수행되었습니다.

#### 2. BodyParts3D (Anatomical 3D Models)

Copyright: © The Database Center for Life Science (DBCLS)

License: Creative Commons Attribution 4.0 International (CC BY 4.0)

Changes: 프로젝트의 목적(ROS 2 시뮬레이션 및 FK 연산)에 맞춰 원본 메쉬의 스케일 조정, 좌표축 변경 및 URDF 호환을 위한 리깅(Rigging) 작업이 수행되었습니다.

#### 3. Orca Hand (Soft Tactile Gripper)

Copyright: © Soft Robotics Lab (SRL), ETH Zurich

License: MIT License

Source: GitHub - [orcahand_description](https://github.com/orcahand/orcahand_description)
