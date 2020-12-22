FROM osrf/ros:noetic-desktop-full

## install ros packages
RUN apt-get update && apt-get install -y \
    apt-utils \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

## set shell to fish because we're going to use this image for development and bash is rarted
RUN apt-get update && apt-get install -y \
    fish \
    wget \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

SHELL ["fish", "--command"]
RUN chsh -s /usr/bin/fish
ENV SHELL /usr/bin/fish
ENV LANG=C.UTF-8 LANGUAGE=C.UTF-8 LC_ALL=C.UTF-8
ENTRYPOINT [ "fish" ]

RUN wget https://raw.githubusercontent.com/edc/bass/master/functions/bass.fish -O /usr/share/fish/functions/bass.fish
RUN wget https://raw.githubusercontent.com/edc/bass/master/functions/__bass.py -O /usr/share/fish/functions/__bass.py

## create ros workspace
ENV ROS_WS /root/catkin_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS
RUN git -C src clone \
    https://github.com/nikisalli/nightmare-software.git

## install ros package dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install -y \
      --from-paths \
        src/nightmare-software \
      --ignore-src && \
    rm -rf /var/lib/apt/lists/*
RUN pip3 install osrf-pycommon

## build ros package source
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO && \
    catkin build

## source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/devel/setup.bash"' \
      /ros_entrypoint.sh

## prepare fish configuration
RUN echo 'bass source /opt/ros/$ROS_DISTRO/setup.bash' >> /etc/fish/config.fish && \
    echo 'bass source $ROS_WS/devel/setup.bash' >> /etc/fish/config.fish && \
    echo 'set -x ROS_IP localhost' >> /etc/fish/config.fish && \
    echo 'set -x ROS_HOSTNAME nikpc' >> /etc/fish/config.fish && \
    echo 'set -x ROS_MASTER_URI http://localhost:11311' >> /etc/fish/config.fish
