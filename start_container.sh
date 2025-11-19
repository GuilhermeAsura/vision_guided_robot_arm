#!/bin/bash
# Script para executar o container

# Configurações do container
CONTAINER_NAME="trainee_container"
IMAGE_NAME="edrom_image"
WORKSPACE_DIR=$(pwd)
DISPLAY_SETTING=$DISPLAY

# Executar o container
docker run -e "TERM=xterm-256color" -t --rm -it \
  --name "$CONTAINER_NAME" \
  --user ros \
  --network=host \
  --ipc=host \
  -v "$WORKSPACE_DIR:/trainee" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --env=DISPLAY="$DISPLAY_SETTING" \
  --env USER=ros \
  -v /dev:/dev \
  --device-cgroup-rule='c *:* rmw' \
  "$IMAGE_NAME"
