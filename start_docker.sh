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

# Start container
echo ">> Starting Docker container (using zsh shell)..."
docker run -it --rm \
  --name daliti_container \
  --network host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $CODE_DIR:/ws/src/DaLiTI:rw \
  daliti:latest \
  /usr/bin/zsh

# Cleanup X11 access permissions
echo ">> Cleaning up X11 access permissions..."
xhost -local:docker > /dev/null 2>&1

echo ">> Container exited"
