FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y \
    vim \
    ffmpeg \
    # camera service
    python3-pip \
    python3-catkin-tools \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade opencv-python==4.6.0.66 websockets==10.4 RPi.GPIO pip install pigpio

ENV WORKDIRECTORY="/home/The-Dream-Robot"
WORKDIR ${WORKDIRECTORY}

# Must use 
RUN bash -c  "source /opt/ros/noetic/setup.bash"
RUN echo "source ${WORKDIRECTORY}/devel/setup.bash" >> ~/.bashrc
# Using /bin/bash to keep the container alive
CMD /bin/bash -c "catkin build && source ${WORKDIRECTORY}/devel/setup.bash && /bin/bash"

