#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONTAINER_NAME="atlas_hand"

show_help() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  help     Show this help message"
    echo "  build    Build the Docker image"
    echo "  start    Start the container"
    echo "  enter    Enter the running container"
    echo "  stop     Stop the container"
    echo ""
    echo "Examples:"
    echo "  $0 build    Build image from Dockerfile"
    echo "  $0 start    Start container"
    echo "  $0 enter    Enter the running container"
    echo "  $0 stop     Stop the container"
}

build_image() {
    echo ">>> Building Docker image..."
    docker build -f "${SCRIPT_DIR}/Dockerfile" \
        -t atlas_hand:latest \
        "$(dirname "${SCRIPT_DIR}")"
}

setup_x11() {
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker || true
    else
        echo "Warning: DISPLAY is not set. X11 forwarding will not be available."
    fi
}

start_container() {
    setup_x11

    echo "Starting atlas_hand container..."
    docker compose -f "${SCRIPT_DIR}/docker-compose.yml" up -d
}

enter_container() {
    setup_x11

    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running. Run '$0 start' first."
        exit 1
    fi

    docker exec -it "$CONTAINER_NAME" bash
}

stop_container() {
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running."
        exit 1
    fi

    echo "Warning: This will stop and remove the container. All unsaved data will be lost."
    read -p "Are you sure? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker compose -f "${SCRIPT_DIR}/docker-compose.yml" down
    else
        echo "Operation cancelled."
        exit 0
    fi
}

case "$1" in
    "help")  show_help ;;
    "build") build_image ;;
    "start") start_container ;;
    "enter") enter_container ;;
    "stop")  stop_container ;;
    *)
        echo "Error: Unknown command '${1:-}'"
        show_help
        exit 1
        ;;
esac
