FROM ros:noetic
SHELL [ "/bin/bash", "-c" ]
ENV ROS_HOSTNAME localhost
ENV ROS_MASTER_URI http://localhost:11311
RUN source /opt/ros/noetic/setup.bash
CMD [ "roscore" ]