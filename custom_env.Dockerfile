
# docker build --tag logivations/ros2:latest --file custom_env.Dockerfile ./
# if there is no full_ros_build, then create it with
# docker build --no-cache --tag nav2:full_ros_build --file full_ros_build.Dockerfile ./

# run like this:
# docker run --runtime=nvidia --privileged -e DISPLAY=:0 -e XAUTHORITY=/root/.Xauthority -v /home/logi/.Xauthority:/root/.Xauthority:rw -v /dev:/dev -it --network=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw --name ros2 logivations/ros2:latest /bin/bash

FROM nav2:full_ros_build

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# rclcpp leads to compilation erros, see https://github.com/ros-planning/navigation2/pull/1449
WORKDIR /opt/ros2_ws/src/ros2/rclcpp
RUN git reset --hard 0c66d0c725ad1f977e2b907bc0619d6ceb6de238

# rebuild everything, so that changes will be compatible and reflected (do not skip finished packages here)
WORKDIR /opt/ros2_ws
RUN colcon build --packages-up-to ros2pkg ros2launch ros2run ros2node ros2topic ros2service rviz2 ros2cli nav2_common

WORKDIR /opt/ros2_ws/src/ros-visualization
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && cd turtlebot3_simulations && git checkout ros2
WORKDIR /opt/ros2_ws/src/ros-visualization
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3.git && cd turtlebot3 && git checkout ros2 && rm -rf turtlebot3_navigation2
WORKDIR /opt/ros2_ws/src/ros-visualization
RUN git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git && cd DynamixelSDK && git checkout ros2
WORKDIR /opt/ros2_ws/src/ros-visualization
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && cd turtlebot3_msgs && git checkout ros2
WORKDIR /opt/ros2_ws
RUN colcon build --packages-up-to dynamixel_sdk turtlebot3_gazebo turtlebot3_description turtlebot3_bringup turtlebot3_msgs --packages-skip-build-finished

WORKDIR /toolkit
RUN cd /toolkit/ && \
	wget -q https://download-cf.jetbrains.com/python/pycharm-community-2019.2.5.tar.gz && \
	tar -xzf pycharm-community-2019.2.5.tar.gz && \
    rm -f /toolkit/pycharm-community-2019.2.5.tar.gz

RUN sed -i '1isource /opt/ros2_ws/install/setup.bash;' /root/.bashrc && \
    sed -i '1iexport GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models;' /root/.bashrc && \
    sed -i '1iexport TURTLEBOT3_MODEL=waffle' /root/.bashrc && \
    sed -i '1iexport AMENT_PREFIX_PATH=/opt/ros2_ws/install/costmap_converter:$AMENT_PREFIX_PATH' /root/.bashrc


# TEB local planner + dependencies
RUN apt update && apt install -y libsuitesparse-dev

WORKDIR /opt/ros2_ws/src/overlay/src
RUN git clone https://github.com/rst-tu-dortmund/costmap_converter.git && cd costmap_converter && git checkout ros2
WORKDIR /opt/ros2_ws
RUN colcon build --packages-up-to tf2_eigen costmap_converter costmap_converter_msgs --packages-skip-build-finished

#g20 (has to be compiled manually for now)
WORKDIR /opt/ros2_ws/src/overlay/src
RUN git clone https://github.com/RainerKuemmerle/g2o.git && cd g2o && cmake . && make -j4 && make install

WORKDIR /opt/ros2_ws/src/overlay/src
RUN git clone https://github.com/logivations/teb_local_planner.git && cd teb_local_planner && git checkout eloquent_changes

# navigation2 custom branch

WORKDIR /opt/ros2_ws/src/overlay/src
RUN rm -rf navigation2 && \
  git clone https://github.com/logivations/navigation2.git && \
  cd navigation2 && \
  git checkout test

# no `source` available in sh, which is default for RUN. Use bash -c
WORKDIR /opt/ros2_ws
RUN bash -c "source install/setup.bash && colcon build --packages-select teb_msgs --symlink-install"
RUN bash -c "source install/setup.bash && colcon build --packages-above nav2_common teb_local_planner --symlink-install"

RUN apt install -y tmux

