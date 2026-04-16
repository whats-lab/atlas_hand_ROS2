# AGA Hand Docker 환경

## 구조

```
docker/
├── Dockerfile      # ros:jazzy-ros-base 기반 빌드 이미지
├── docker.sh       # 빌드 / 셸 진입 스크립트
└── README.md
```

## 사전 요구사항

- Docker Engine 24.0+

---

## 빠른 시작

호스트 셸에서 아래 환경변수를 먼저 설정해야 합니다.

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

```bash
# 1. 이미지 빌드 (모든 의존성 포함)
./docker/docker.sh build

# 2. 컨테이너 접속
./docker/docker.sh enter
```

`docker.sh enter`는 컨테이너 내부에도 `FASTDDS_BUILTIN_TRANSPORTS`를 전달하며,
호스트에서 미설정 시 기본값 `UDPv4`를 사용합니다.

---

## 명령어

| 명령어 | 설명 |
|--------|------|
| `./docker/docker.sh build` | 전체 이미지 빌드 (dex-retargeting, pin 포함) |
| `./docker/docker.sh enter` | 인터랙티브 셸 진입 |

---

## Dockerfile 구성

```
ros:jazzy-ros-base
  └── rosdep install     # ROS 의존성 자동 설치
  └── pip install        # requirements.txt 전체 패키지 설치
  └── colcon build       # atlas_hand 패키지 빌드
  └── .bashrc 설정       # source, alias cb, ROS_DOMAIN_ID=30
```
