FROM osrf/ros:noetic-desktop-full

## install things
RUN apt-get update && apt-get install -y \
    # os things
    apt-utils \
    git \
    python3-pip \
    wget \
    # ros things
    python3-catkin-tools \
    # video things
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

## create ros workspace
ENV ROS_WS /root/catkin_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS
RUN git -C src clone \
    --branch features/movements \
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
RUN pip3 install -r src/nightmare-software/requirements.txt

## build ros package source
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO && \
    catkin build

## source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$ROS_WS/devel/setup.bash"' \
      /ros_entrypoint.sh

## Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

## run system upgrade to solve ros bugs https://answers.ros.org/question/360537/unknown-error-handler-name-rosmsg/?answer=360643#post-id-360643
RUN apt-get update && \
    apt-get upgrade -y && \
    rm -rf /var/lib/apt/lists/*

## change all the code below to install your preferred shell
## set shell to fish because we're going to use this image for development and bash is rarted
RUN apt-get update && apt-get install -y \
    fish \
    && rm -rf /var/lib/apt/lists/*

SHELL ["fish", "--command"]
RUN chsh -s /usr/bin/fish
ENV SHELL /usr/bin/fish
ENV LANG=C.UTF-8 LANGUAGE=C.UTF-8 LC_ALL=C.UTF-8
ENTRYPOINT [ "fish" ]

RUN wget https://raw.githubusercontent.com/edc/bass/master/functions/bass.fish -O /usr/share/fish/functions/bass.fish
RUN wget https://raw.githubusercontent.com/edc/bass/master/functions/__bass.py -O /usr/share/fish/functions/__bass.py

## prepare fish configuration
## source this files and set this env vars in your shell's startup script
RUN echo 'bass source /opt/ros/$ROS_DISTRO/setup.bash' >> /etc/fish/config.fish && \
    echo 'bass source $ROS_WS/devel/setup.bash' >> /etc/fish/config.fish && \
    echo 'set -x ROS_IP localhost' >> /etc/fish/config.fish && \
    echo 'set -x ROS_HOSTNAME localhost' >> /etc/fish/config.fish && \
    echo 'set -x ROS_MASTER_URI http://localhost:11311' >> /etc/fish/config.fish
