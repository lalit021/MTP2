#!/bin/bash

# Variables
CONTAINER_NAME="ros_melodic_container"
NEW_IMAGE_NAME="my_ros_container:latest"
RESTART_CONTAINER=${1:-no}

echo "Committing container '$CONTAINER_NAME' to image '$NEW_IMAGE_NAME'..."

# Commit the container to a new image
docker commit $CONTAINER_NAME $NEW_IMAGE_NAME
if [ $? -ne 0 ]; then
    echo "Failed to commit container '$CONTAINER_NAME'."
    exit 1
fi

echo "Container committed successfully to image '$NEW_IMAGE_NAME'."

# Restart the container if requested
if [ "$RESTART_CONTAINER" == "restart" ]; then
    echo "Restarting container '$CONTAINER_NAME'..."
    docker stop $CONTAINER_NAME
    docker start $CONTAINER_NAME
    if [ $? -ne 0 ]; then
        echo "Failed to restart container '$CONTAINER_NAME'."
        exit 1
    fi
    echo "Container restarted successfully."
else
    echo "Skipping container restart."
fi
