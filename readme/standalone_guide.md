# Standalone 실행 가이드

ROS 2 없이 손 추적 장치와 직접 연결하여 시각화하거나 IK 결과를 확인할 수 있습니다.

## 의존성

```bash
pip install -r requirements.txt
```

| 패키지            | 용도           |
| ----------------- | -------------- |
| `numpy`           | 수치 연산      |
| `scipy`           | 회전 변환      |
| `pin`             | Pinocchio (FK) |
| `dex-retargeting` | IK 최적화      |
| `rerun-sdk`       | 3D 시각화      |
| `python-osc`      | OSC UDP 통신 (Atlas) |

---

## 설치

```bash
git clone https://github.com/whats-lab/atlas_hand_ROS2 
cd atlas_hand_ROS2
pip install -r requirements.txt
```

---

## 실행

### 3D 시각화 (Rerun)

```bash
# 기본 (Atlas 글러브, 로컬 뷰어)
python standalone/visualize.py left
python standalone/visualize.py right

# 외부 Rerun 뷰어 연결
python standalone/visualize.py left connect

# Meta Quest
python standalone/visualize.py left spawn meta_quest
python standalone/visualize.py right spawn meta_quest
```

인자 순서: `[left|right] [spawn|connect] [atlas|meta_quest]`

### FK → IK 리타겟팅 (콘솔 출력)

```bash
# 기본 (Atlas 글러브)
python standalone/retarget.py left
python standalone/retarget.py right

# 로봇 핸드 설정 지정
python standalone/retarget.py left orca_hand
python standalone/retarget.py right robotis_hx5

# Meta Quest
python standalone/retarget.py left base meta_quest
python standalone/retarget.py right orca_hand meta_quest
```

인자 순서: `[left|right] [robot_config] [atlas|meta_quest]`

---

## 입력 소스 설정

### Atlas 글러브 OSC 기본값

| 상수              | 기본값      | 설명              |
| ----------------- | ----------- | ----------------- |
| `OSC_LISTEN_IP`   | `0.0.0.0`   | 수신 IP           |
| `OSC_SERVER_PORT` | `4040`      | OSC 수신 포트     |
| `OSC_TARGET_IP`   | `127.0.0.1` | 디바이스 IP       |
| `OSC_CLIENT_PORT` | `4042`      | 하트비트 포트     |

### Meta Quest UDP 기본값

| 상수               | 기본값 | 설명                    |
| ------------------ | ------ | ----------------------- |
| `QUEST_UDP_PORT`   | `9090` | UDP 데이터 수신 포트    |
| `QUEST_DISC_PORT`  | `9001` | MXFIND 자동 발견 포트   |

기본값 변경이 필요하면 [atlas_hand_core/config.py](../atlas_hand_core/config.py)를 수정하세요.

---

## 커스텀 처리

`standalone/retarget.py`의 `on_result()` 함수를 수정하면 IK 결과를 원하는 방식으로 처리할 수 있습니다.

```python
def on_result(joint_names: list, qpos: np.ndarray):
    # 여기서 joint angles를 직렬 포트, 소켓 등으로 전송
    ...
```

---

## 새 로봇 핸드 추가

상세 튜토리얼: [adding_hand_config.md](adding_hand_config.md)

1. `HandConfig`를 상속하는 클래스를 [atlas_hand_core/hand_configs.py](../atlas_hand_core/hand_configs.py)에 구현
2. `CONFIG_REGISTRY`에 키 등록
3. 실행 시 `robot_config` 인자로 선택

```bash
python standalone/retarget.py left <key>
python standalone/visualize.py left spawn  # visualize는 base 모델 URDF 사용
```
