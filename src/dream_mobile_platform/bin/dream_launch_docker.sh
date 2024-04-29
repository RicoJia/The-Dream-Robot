#!/bin/bash

# Get the GID for the 'dialout' group
dialout_gid=$(getent group dialout | cut -d: -f3)

echo "dialout gid: ${dialout_gid}"

# Check if we got the GID
if [ -z "$dialout_gid" ]; then
    echo "Failed to get GID for dialout group"
    exit 1
fi

# Now, run the Docker container with the correct group-add parameter
sudo docker run --name my_ros_container --rm \
    -e DISPLAY=$DISPLAY \
    --device /dev/ttyUSB0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -e XAUTHORITY=/root/.Xauthority \
    -v /home/ricojia/software/The-Dream-Robot/:/home/The-Dream-Robot \
    -v ~/.ssh:/root/.ssh \
    --network="host" \
    --privileged \
    --group-add $dialout_gid \
    -it ricojia/rpi-dream-mobile-platform



