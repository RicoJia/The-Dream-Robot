FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y \
    vim \
    ffmpeg \
    # camera service
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/The-Dream-Robot

