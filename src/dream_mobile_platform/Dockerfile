# Test Procedure:
# docker build -t dream-mobile-platform .
# If you'd like the simulation args:
# docker build --build_args BUILD_SIM=true -t dream-mobile-platform .
# sudo docker run --name my_ros_container --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v $HOME/.Xauthority:/root/.Xauthority -e XAUTHORITY=/root/.Xauthority -v /home/rjia/file_exchange_port/The-Dream-Robot/:/home/The-Dream-Robot -v ~/.ssh:/root/.ssh   -it   --network="host"   --privileged  dream-mobile-platform

# Build Procedure:
# On dev machine:
# docker buildx build --platform linux/arm64/v8 -t rpi-dream-mobile-platform . --load
# If you see error that cannot pull images, then do:
# docker kill buildx_buildkit_rpi-builder0
# docker tag rpi-dream-mobile-platform:latest ricojia/rpi-dream-mobile-platform:latest
# docker push ricojia/rpi-dream-mobile-platform:latest

# On rpi: 
# make sure you are on the same network as rpi
# sudo docker pull ricojia/rpi-dream-mobile-platform:latest
# sudo docker run --name my_ros_container --rm -e DISPLAY=$DISPLAY --device /dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v $HOME/.Xauthority:/root/.Xauthority -e XAUTHORITY=/root/.Xauthority -v /home/ricojia/software/The-Dream-Robot/:/home/The-Dream-Robot -v ~/.ssh:/root/.ssh   -it   --network="host"   --privileged   ricojia/rpi-dream-mobile-platform

FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y \
    vim \
    ffmpeg \
    # camera service
    python3-pip \
    python3-catkin-tools \
    byobu \
    v4l-utils \
    wget \
    unzip \
    git \
    # useful for git complete
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

# TODO
ARG BUILD_SIM="false"
RUN if [ "$BUILD_SIM" = "true" ]; then \
    apt-get update && apt-get install -y \
        ros-noetic-joint-state-publisher \
        ros-noetic-robot-state-publisher \
        ros-noetic-rviz \
        ros-noetic-xacro \
        ros-noetic-tf2 \
        ros-noetic-tf \
        ros-noetic-tf2-eigen \
        ros-noetic-tf2-ros \ 
        gazebo11 \
        ros-noetic-gazebo-ros \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gazebo-ros-control \
        ros-noetic-joint-state-publisher-gui \
        && rm -rf /var/lib/apt/lists/*; fi

RUN pip3 install --upgrade opencv-python==4.6.0.66 \
 websockets==10.4 \
 pynput==1.7.6 \
 numpy==1.22.4 RPi.GPIO posix_ipc  numba==0.56.0 \
 adafruit_rplidar pyserial==3.5

RUN wget https://github.com/joan2937/pigpio/archive/master.zip && unzip master.zip && cd pigpio-master && make && sudo make install

# Making the image have the same user privilege as the host user
# Unique numbers for each group and user. 1000 is the first regular number
ARG UNAME="dream_robot"
# Sets default value if docker build --build-arg UID=1000 is not provided.
# So, one caveat of this script is on Rpi, UID, GID have to be 1000.
ARG UID=1000
ARG GID=1000
RUN groupadd -f -g $GID $UNAME
RUN id -u $UNAME || useradd -m -u $UID -g $GID -s /bin/bash $UNAME
# RUN adduser --disabled-password --gecos '' ${UNAME}
RUN adduser ${UNAME} sudo
# adding UNAME to sudoers
RUN echo "${UNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $UNAME

# Env variables will persist across images
ENV WORKDIRECTORY="/home/The-Dream-Robot"
ENV PATH="${WORKDIRECTORY}/src/dream_mobile_platform/bin:${PATH}"
ENV SKIP_OFFLINE_PACKAGES="true"
# ENV TERM="xterm-256color"
WORKDIR ${WORKDIRECTORY}

# Must use 
RUN bash -c  "source /opt/ros/noetic/setup.bash" && \
    echo '[[ -f "${WORKDIRECTORY}/devel/setup.bash" ]] && source "${WORKDIRECTORY}/devel/setup.bash"' >> ~/.bashrc && \
    echo 'source "${WORKDIRECTORY}/src/dream_mobile_platform/dream_bashrc.sh"' >> ~/.bashrc
# Using /bin/bash to keep the container alive
# ENTRYPOINT or command will create a new shell. That shell gets the parent shells' env var as indicated in CMD, but not functions. 
# That's why we do NOT put feature flags in a dream_bashrc.sh
# ENTRYPOINT ["/bin/bash", "-c", "source ${WORKDIRECTORY}/src/dream_mobile_platform/dream_feature_flags && exec \"$@\"", "--"]
CMD /bin/bash

