# Test Procedure:
# docker build -t dream-mobile-platform .
# docker run --name dream-mobile-platform --rm -p 3000:3000 -v $(pwd):/home/The-Dream-Robot/src/dream_mobile_platform -it dream-mobile-platform
# Build Procedure:
# On dev machine:
# docker buildx build --platform linux/arm64/v8 -t rpi-dream-mobile-platform . --load
# If you see error that cannot pull images, then do:
# docker kill buildx_buildkit_rpi-builder0
# docker tag rpi-dream-mobile-platform:latest ricojia/rpi-dream-mobile-platform:latest
# docker push ricojia/rpi-dream-mobile-platform:latest
# On rpi: 
# sudo docker pull ricojia/rpi-dream-mobile-platform:latest
# sudo docker run \
#   --name my_ros_container \
#   --rm \
#   -v /home/ricojia/software/The-Dream-Robot/:/home/The-Dream-Robot \
#   -v ~/.ssh:/root/.ssh \
#   -it \
#   --network="host" \
#   --privileged \
#   ricojia/rpi-dream-mobile-platform

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

RUN pip3 install --upgrade opencv-python==4.6.0.66 \
 websockets==10.4 \
 numpy==1.22.4 RPi.GPIO posix_ipc  numba==0.56.0

RUN wget https://github.com/joan2937/pigpio/archive/master.zip && unzip master.zip && cd pigpio-master && make && sudo make install

# Making the image have the same user privilege as the host user
# Unique numbers for each group and user. 1000 is the first regular number
ARG UNAME="dream_robot"
ARG UID=1000
ARG GID=1000
RUN adduser --disabled-password --gecos '' ${UNAME}
RUN adduser ${UNAME} sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
# RUN groupadd -f -g $GID $UNAME && \
# id -u $UNAME || useradd -m -u $UID -g $GID -s /bin/bash $UNAME

# This could be a security concern, but it's for convenience
# Set the image to run as the user
USER $UNAME

# Env variables will persist across images
ENV WORKDIRECTORY="/home/The-Dream-Robot"
ENV PATH="${WORKDIRECTORY}/src/dream_mobile_platform/bin:${PATH}"
# ENV TERM="xterm-256color"

WORKDIR ${WORKDIRECTORY}

# Must use 
RUN bash -c  "source /opt/ros/noetic/setup.bash" && \
    echo '[[ -f "${WORKDIRECTORY}/devel/setup.bash" ]] && source "${WORKDIRECTORY}/devel/setup.bash"' >> ~/.bashrc && \
    echo 'source "${WORKDIRECTORY}/src/dream_mobile_platform/dream_bashrc.sh"' >> ~/.bashrc
# Using /bin/bash to keep the container alive
CMD /bin/bash

