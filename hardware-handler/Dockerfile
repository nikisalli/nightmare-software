FROM ubuntu:latest
SHELL [ "/bin/bash", "-c" ]
ENV ROS_HOSTNAME localhost
ENV ROS_MASTER_URI http://localhost:11311
RUN apt update && apt install -y \
	catkin \
	gcc \
	g++ \
	python3-rospy \
	ros-std-msgs

COPY nightmare_hardware_handler /nightmare_hardware_handler
RUN cd nightmare_hardware_handler && \
	catkin_make

CMD [ "/bin/bash", "-c", "cd /nightmare_hardware_handler && source devel/setup.bash && ROS_MASTER_URI=http://127.0.0.1:11311 rosrun listener listener.py" ]