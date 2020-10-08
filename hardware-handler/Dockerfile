FROM ubuntu:latest
SHELL [ "/bin/bash", "-c" ]

RUN apt update && apt install -y \
	catkin \
	gcc \
	g++ \
	python3-rospy \
	ros-std-msgs

COPY nightmare-hardware-handler /nightmare-hardware-handler
RUN cd nightmare-hardware-handler && \
	catkin_make

CMD [ "/bin/bash", "-c", "cd /nightmare-hardware-handler && source devel/setup.bash && ROS_MASTER_URI=http://127.0.0.1:11311 rosrun listener listener.py" ]