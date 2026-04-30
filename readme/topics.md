# ROS 2 토픽 레퍼런스

## 발행 토픽

| 토픽 | 타입 | 발행 노드 | 설명 |
|------|------|-----------|------|
| `/left_hand/quaternions`  | `std_msgs/Float32MultiArray` | osc_receiver | 왼손 17관절 쿼터니언 (68 floats) |
| `/right_hand/quaternions` | `std_msgs/Float32MultiArray` | osc_receiver | 오른손 17관절 쿼터니언 (68 floats) |
| `/left_hand/connected`    | `std_msgs/Bool`              | osc_receiver | 왼손 디바이스 연결 상태 |
| `/right_hand/connected`   | `std_msgs/Bool`              | osc_receiver | 오른손 디바이스 연결 상태 |
| `/joint_states`           | `sensor_msgs/JointState`     | retarget     | 로봇 관절 각도 |

## 구독 토픽

| 토픽 | 타입 | 구독 노드 | 설명 |
|------|------|-----------|------|
| `/left_hand/haptic`      | `std_msgs/Int32MultiArray` | osc_receiver | 왼손 햅틱 on 명령 (int×5) |
| `/right_hand/haptic`     | `std_msgs/Int32MultiArray` | osc_receiver | 오른손 햅틱 on 명령 (int×5) |
| `/left_hand/haptic/off`  | `std_msgs/Empty`           | osc_receiver | 왼손 햅틱 끄기 트리거 |
| `/right_hand/haptic/off` | `std_msgs/Empty`           | osc_receiver | 오른손 햅틱 끄기 트리거 |

---

## 데이터 포맷

### 쿼터니언 (`Float32MultiArray`, 68 floats)

```
data = [x0,y0,z0,w0,  x1,y1,z1,w1,  ...,  x16,y16,z16,w16]
         sensor[0]=손목  /  sensor[1~16]=관절
```

pinky_0 (index 14) 는 제외되어 18관절 → 17센서로 축소됩니다.

### 햅틱 on (`Int32MultiArray`, 5 ints)

```
data = [thumb, index, middle, ring, pinky]   # 액추에이터 인덱스 0~4
```

`/left_hand/haptic` 또는 `/right_hand/haptic`에 발행하면 디바이스로
`/left/hapt/set` (타입 `"9"`) 또는 `/right/hapt/set` (타입 `"10"`) OSC 패킷을 전송합니다.
`/left/hapt/ret/get` 응답값과 일치할 때까지 0.5초 간격으로 재전송합니다.

해당 손이 연결되지 않은 경우(`/left_hand/connected` = false) 전송을 생략하고 경고를 출력합니다.

### 햅틱 off (`Empty`)

`/left_hand/haptic/off` 또는 `/right_hand/haptic/off`에 발행하면
`[0, 0, 0, 0, 0]`을 전송하여 햅틱을 끕니다.

---

## 예시

```bash
# 햅틱은 0단계부터 4단계 까지 있으며 On/Off 방식으로 구현되어있음.
# 왼손 햅틱 on (thumb=1, index=2, middle=0, ring=3, pinky=0)
ros2 topic pub -t 1 /left_hand/haptic std_msgs/Int32MultiArray "data: [10, 20, 0, 5, 0]"

# 왼손 햅틱 off
ros2 topic pub -t 1 /left_hand/haptic/off std_msgs/Empty "{}"

# 연결 상태 확인
ros2 topic echo /left_hand/connected
ros2 topic echo /right_hand/connected
```
