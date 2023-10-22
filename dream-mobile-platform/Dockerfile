FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y \
    vim \
    ffmpeg \
    # camera service
    python3-pip \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade opencv-python==4.6.0.66 websockets==10.4 RPi.GPIO pip install pigpio

WORKDIR /home/The-Dream-Robot

