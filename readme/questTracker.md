# questTracker — Quest 컨트롤러 / 핸드 트래킹 수신 라이브러리

Meta Quest APK(`PoseDataTracker`)가 OSC 로 보내는 **컨트롤러 pose** 와 **핸드 트래킹**
(손목 root + 16관절) 데이터를 PC 에서 수신·파싱하는 라이브러리입니다.
자동 발견(auto-discovery)으로 IP 설정 없이 붙습니다.

> **이 패키지에서의 위치**: 이 파싱 계층(`OscReceiver`/`PoseTracker`/`ControllerTracker`/
> `HandTracker`)은 `atlas_hand_core/sources/meta_quest.py` 안에 통합되어 있으며,
> 같은 파일의 `MetaQuestSource(HandInputSource)` 어댑터가 이를 감싸 노드에 17×4 쿼터니언을
> push 합니다. 아래 문서는 파싱 계층 자체의 레퍼런스입니다(단독으로도 사용 가능).

---

## 목차
1. [빠른 시작](#빠른-시작)
2. [구성 파일](#구성-파일)
3. [설치 & 준비](#설치--준비)
4. [클래스 구조](#클래스-구조)
5. [API 레퍼런스](#api-레퍼런스)
6. [OSC 프로토콜](#osc-프로토콜)
7. [자주 묻는 것 / 트러블슈팅](#자주-묻는-것--트러블슈팅)

---

## 빠른 시작

```bash
pip install numpy python-osc          # PC 의존성 (최초 1회)
adb install "PoseDataTracker 1.0.0.apk"   # Quest 에 송신 앱 설치
python questTracker.py --track controller  # 수신 확인 (controller | hand)
```

```python
from questTracker import ControllerTracker

ctrl = ControllerTracker()
ctrl.start()                          # OSC 서버 + 자동 발견 시작 (논블로킹)
while True:
    if ctrl.connected("right"):
        pos = ctrl.get_pos("right")   # np.ndarray([x, y, z])
        rot = ctrl.get_rot("right")   # np.ndarray([x, y, z, w])
```

---

## 구성 파일

| 파일 | 설명 |
|------|------|
| `questTracker.py` | 수신·파싱 라이브러리 (의존: numpy, python-osc) |
| `PoseDataTracker 1.0.0.apk` | Quest 에 설치하는 송신 앱 |
| `README.md` | 이 문서 |

---

## 설치 & 준비

### PC
```bash
pip install numpy python-osc
```

### Quest
```bash
adb install "PoseDataTracker 1.0.0.apk"    # 또는 SideQuest 로 설치
```
설치 후 헤드셋에서 앱을 실행하면 송신을 시작합니다.

### 네트워크
- PC 와 Quest 가 **같은 서브넷**(같은 공유기/AP)에 있어야 합니다.
- PC 방화벽에서 **UDP 9000**(데이터), **UDP 9001**(자동 발견) 인바운드를 허용하세요.
- 자동 발견을 쓰지 않고 고정 IP 로 직접 쏘는 경우 `disc_port=0` 으로 두면 됩니다.

---

## 클래스 구조

전송(OSC 서버 + 자동 발견)과 파싱을 상속으로 분리했습니다. 컨트롤러와 핸드 root 는
주소 접두어(`/controller` vs `/hand`)만 다르고 pos/rot 값 형식이 같으므로, 그 공통
로직을 `PoseTracker` 로 한 번만 구현하고 두 트래커가 상속합니다.

```
OscReceiver          ── 전송: OSC UDP 서버 생명주기 + 자동 발견(MXFIND→MXHELLO)
└ PoseTracker        ── pos/rot 저장·제공: get_pos / get_rot / connected
   ├ ControllerTracker  ── /controller/<side>/pos·rot
   └ HandTracker        ── /hand/<side>/pos·rot  +  joints/pos·rot
                           (get_joint_pos / get_joint_rot 추가)
```

- **`OscReceiver`** — 서버를 띄우고 자동 발견 스레드를 돌린다. 어떤 OSC 주소를 받을지는
  자식의 `_register()` 훅에 위임(Template Method).
- **`PoseTracker`** — `PREFIX` 하나만 바꾸면 되는 root pose 트래커. 좌·우를 모두 보관.
- **`ControllerTracker` / `HandTracker`** — 실제로 쓰는 두 클래스. 각각 좌·우(`"left"`/`"right"`)
  를 모두 들고 있으므로, 트래커 하나로 양손을 다 읽습니다.

> 두 트래커는 형제입니다(하나가 다른 하나를 상속하지 않음). "핸드는 컨트롤러가 아니다"라는
> 관계를 지키기 위해 공통점을 부모 `PoseTracker` 로 뽑았습니다.

---

## API 레퍼런스

### 생성자 인자 (공통)

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `listen_ip` | `"0.0.0.0"` | 바인드할 로컬 IP |
| `osc_port` | `9000` | OSC 데이터 수신 포트 |
| `disc_port` | `9001` | 자동 발견 포트. `0` 이면 발견 비활성 |
| `stale_timeout` | `0.0` | `>0` 이면 그 시간(초) 이상 패킷이 없을 때 `connected()` 가 `False` |
| `verbose` | `False` | (`HandTracker` 만) joints 수신 디버그 로그 |

### 공통 메서드 (`ControllerTracker`, `HandTracker`)

| 메서드 | 반환 | 설명 |
|--------|------|------|
| `start()` | `None` | 수신 시작. 논블로킹(백그라운드 스레드). 중복 호출 안전(idempotent) |
| `stop()` | `None` | 수신 종료 |
| `get_pos(side)` | `np.ndarray(3,)` | `side` 의 위치 `[x, y, z]`. 핸드는 손목(root) 위치 |
| `get_rot(side)` | `np.ndarray(4,)` | `side` 의 회전 쿼터니언 `[x, y, z, w]` (정규화됨) |
| `connected(side)` | `bool` | `side` 로 패킷을 받은 적이 있는지(+`stale_timeout` 적용) |

- `side` 는 `"left"` 또는 `"right"`.
- 모든 게터는 **내부 배열의 복사본**을 반환하므로 호출자가 수정해도 안전합니다.
- 좌표는 모두 **HMD 로컬** 기준입니다.

### `HandTracker` 전용 메서드

| 메서드 | 반환 | 설명 |
|--------|------|------|
| `get_joint_pos(side)` | `np.ndarray(16, 3)` | 16관절 위치. 행 순서 = `JOINT_ORDER` |
| `get_joint_rot(side)` | `np.ndarray(16, 4)` | 16관절 회전 쿼터니언(정규화됨). 행 순서 = `JOINT_ORDER` |

### 반환값 규약

- 아직 패킷을 못 받았으면 게터는 **중립값**을 돌려줍니다: `pos = [0,0,0]`,
  `rot = [0,0,0,1]`(단위 쿼터니언), joints 는 그에 준하는 중립 배열.
  → 데이터 유효성은 항상 `connected(side)` 로 판정하세요.

### `JOINT_ORDER` (16관절 순서)

`get_joint_pos` / `get_joint_rot` 의 행 순서입니다.

```
0  thumb_cmc0    4  index_mcp    7  middle_mcp   10 ring_mcp    13 pinky_mcp
1  thumb_cmc1    5  index_pip    8  middle_pip   11 ring_pip    14 pinky_pip
2  thumb_mcp     6  index_dip    9  middle_dip   12 ring_dip    15 pinky_dip
3  thumb_ip
```

### 사용 예 — 핸드

```python
from questTracker import HandTracker

hand = HandTracker(stale_timeout=0.5)   # 0.5초 이상 끊기면 disconnected 처리
hand.start()

if hand.connected("left"):
    wrist_pos = hand.get_pos("left")        # (3,)   손목 root 위치
    wrist_rot = hand.get_rot("left")        # (4,)   손목 root 회전
    joints_p  = hand.get_joint_pos("left")  # (16,3) 관절 위치
    joints_r  = hand.get_joint_rot("left")  # (16,4) 관절 회전
    index_tip_rot = joints_r[6]             # JOINT_ORDER[6] = index_dip

hand.stop()
```

---

## OSC 프로토콜

### 자동 발견 (discovery)
1. Quest 앱이 UDP **9001** 로 `MXFIND` 브로드캐스트를 쏜다.
2. PC(`OscReceiver`)가 받으면 `MXHELLO` + 데이터 포트(2바이트 LE)로 응답한다.
3. Quest 가 그 포트(기본 9000)로 데이터 송신을 시작한다.

### 데이터 주소 (모두 HMD 로컬 좌표)

| 주소 | 페이로드 | 파싱 메서드 |
|------|----------|-------------|
| `/controller/<left\|right>/pos` | `float x, y, z` | `ControllerTracker.get_pos` |
| `/controller/<left\|right>/rot` | `float x, y, z, w` | `ControllerTracker.get_rot` |
| `/hand/<left\|right>/pos` | `float x, y, z` (손목 root) | `HandTracker.get_pos` |
| `/hand/<left\|right>/rot` | `float x, y, z, w` | `HandTracker.get_rot` |
| `/hand/<left\|right>/joints/pos` | `float[48]` = 16×3 | `HandTracker.get_joint_pos` |
| `/hand/<left\|right>/joints/rot` | `float[64]` = 16×4 | `HandTracker.get_joint_rot` |

---

## 자주 묻는 것 / 트러블슈팅

**Q. 컨트롤러와 핸드를 동시에 받고 싶어요.**
두 트래커는 기본으로 같은 포트(9000)를 씁니다. 한 프로세스에서 동시에 띄우려면 한쪽의
`osc_port` 를 바꾸고 **APK 송신 포트도 맞춰야** 합니다. 보통은 필요한 트래커 하나만 씁니다.

**Q. `connected()` 가 계속 `False` 예요.**
- PC·Quest 가 같은 서브넷인지, 방화벽에서 UDP 9000/9001 이 열렸는지 확인.
- `python questTracker.py --track ...` 로 원시 수신이 되는지 먼저 확인.
- 자동 발견을 안 쓰면 APK 에서 PC IP·포트를 수동 지정하고 `disc_port=0`.

**Q. 값이 이상해요(축이 뒤바뀜 등).**
이 모듈은 **파싱만** 합니다(좌표 변환 없음). 좌표계 정합/리매핑은 사용하는 쪽에서 처리하세요.

**Q. 스레드 안전한가요?**
네. 내부 상태는 락으로 보호되고 게터는 복사본을 반환합니다. 아무 스레드에서나 게터 호출 가능.

**Q. 데이터 입력이 자꾸 끊겨요**
동일한 네트워크내에서 2개이상의 기기가 APK를 연결할시 서로 Confuse 되어서 문제가 발생 할 수 있습니다.