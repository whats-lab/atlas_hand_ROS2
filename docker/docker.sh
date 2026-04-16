#!/bin/bash
# 사용법:
#   ./docker/docker.sh build         이미지 빌드 
#   ./docker/docker.sh enter         인터랙티브 셸

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE="atlas_hand:latest"
COLCON_WS="/root/ros2_ws"
FASTDDS_TRANSPORT="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

case "$1" in

  build)
    echo ">>> 이미지 빌드 (모든 의존성 포함)"
    docker build -f "${SCRIPT_DIR}/Dockerfile" \
      -t "$IMAGE" "$REPO_ROOT"
    ;;

  enter)
    CONTAINER="atlas_hand_dev"

    fi
    
    if docker ps -q -f name="^${CONTAINER}$" | grep -q .; then
      docker exec -it \
        -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_TRANSPORT}" \
        "$CONTAINER" bash
    else
      docker run --rm -it --name "$CONTAINER" \
        --ipc=host \
        --network host \
        -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_TRANSPORT}" \
        -e DISPLAY=:0 \
        -v /:/host \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$REPO_ROOT:$COLCON_WS/src/atlas_hand" \
        --privileged \
        "$IMAGE" bash
    fi
    ;;

  *)
    echo "사용법: $0 {build | enter}"
    exit 1
    ;;

esac
