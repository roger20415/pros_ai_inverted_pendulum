#!/bin/bash

NETWORK_NAME="inverted_pendulum_bridge_network"
IMAGE_NAME="registry.screamtrumpet.csie.ncku.edu.tw/unity_env/pros_rl_image"
IMAGE_TAG="latest"
ENV_FILE="./.env"
WORKSPACE_PATH="$(pwd)/rl_training_pipeline"

# ⬇ 自動設定 DISPLAY 變數（適用於 WSL2 + XLaunch）
export DISPLAY=:0
# export LIBGL_ALWAYS_INDIRECT=1
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

create_network() {
    echo "Checking if network $NETWORK_NAME exists..."
    if ! docker network ls | grep -q "$NETWORK_NAME"; then
        echo "Creating network $NETWORK_NAME..."
        docker network create $NETWORK_NAME || { echo "Failed to create network $NETWORK_NAME"; exit 1; }
    else
        echo "Network $NETWORK_NAME already exists."
    fi
}

build_image() {
    echo "Checking if image $IMAGE_NAME exists..."
    if ! docker images | grep -q "$IMAGE_NAME"; then
        echo "Image $IMAGE_NAME not found, building it..."
        docker build -t $IMAGE_NAME . || { echo "Failed to build image $IMAGE_NAME"; exit 1; }
    else
        echo "Image $IMAGE_NAME already exists."
    fi
}


run_container() {
    echo "Running the Docker container with the image $IMAGE_NAME..."
    xhost +
    docker run -it --rm --gpus all \
        -v "$WORKSPACE_PATH:/workspaces/rl_training_pipeline" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --network $NETWORK_NAME \
        -p 9090:9090 \
        --env DISPLAY \
        --env NVIDIA_DRIVER_CAPABILITIES=all \
        --env QT_X11_NO_MITSHM=1 \
        --env-file $ENV_FILE \
        $IMAGE_NAME:$IMAGE_TAG /bin/bash || { echo "Failed to run Docker container"; exit 1; }
}

        # --env LIBGL_ALWAYS_INDIRECT=1 \
main() {
    create_network
    build_image
    run_container
}

main
