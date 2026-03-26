#!/bin/bash

# DaLiTI Docker startup script
# Includes code mounting and RViz visualization support

set -e

echo "========================================="
echo "  DaLiTI Docker Container Startup"
echo "========================================="

# Get script directory (i.e., src/DaLiTI)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Allow Docker to access X11 server
echo ">> Configuring X11 access permissions..."
xhost +local:docker > /dev/null 2>&1

# Check DISPLAY environment variable
if [ -z "$DISPLAY" ]; then
    echo "Warning: DISPLAY environment variable not set, RViz may not display"
    export DISPLAY=:0
fi

echo ">> DISPLAY: $DISPLAY"

# Check if image exists
if ! docker image inspect daliti:latest > /dev/null 2>&1; then
    echo "Error: Docker image 'daliti:latest' does not exist"
    echo "Please run first: cd $WORKSPACE_DIR && docker build -t daliti:latest -f src/DaLiTI/Dockerfile ."
    exit 1
fi

# Check code directory
CODE_DIR="$SCRIPT_DIR"
if [ ! -d "$CODE_DIR" ]; then
    echo "Error: Code directory does not exist: $CODE_DIR"
    exit 1
fi

echo ">> Code mount: $CODE_DIR -> /ws/src/DaLiTI"

# Try to enable NVIDIA GPU acceleration when the host and Docker runtime support it.
GPU_ARGS=()
if command -v nvidia-smi > /dev/null 2>&1; then
    if docker run --rm --gpus all --entrypoint /bin/true daliti:latest > /dev/null 2>&1; then
        GPU_ARGS+=(--gpus all)
        GPU_ARGS+=(-e NVIDIA_VISIBLE_DEVICES=all)
        GPU_ARGS+=(-e NVIDIA_DRIVER_CAPABILITIES=all)
        echo ">> NVIDIA GPU support enabled for Docker"
    else
        echo "Warning: NVIDIA GPU detected on host, but Docker GPU runtime is not configured"
        echo "         Container will start without GPU acceleration"
    fi
fi

# Start container
echo ">> Starting Docker container (using zsh shell)..."
docker run -it --rm \
  --name daliti_container \
  --network host \
  --privileged \
  "${GPU_ARGS[@]}" \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $CODE_DIR:/ws/src/DaLiTI:rw \
  daliti:latest \
  /usr/bin/zsh

# Cleanup X11 access permissions
echo ">> Cleaning up X11 access permissions..."
xhost -local:docker > /dev/null 2>&1

echo ">> Container exited"
