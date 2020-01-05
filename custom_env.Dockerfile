FROM nav2:full_ros_build

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /opt/ros2_ws
RUN colcon build --packages-up-to ros2pkg ros2launch ros2run ros2node ros2topic ros2service rviz2 ros2cli --packages-skip-build-finished

WORKDIR /opt/ros2_ws/src/ros2/rclcpp
RUN git reset --hard 0c66d0c725ad1f977e2b907bc0619d6ceb6de238
WORKDIR /opt/ros2_ws
RUN colcon build --packages-up-to rviz2


WORKDIR /opt/ros2_ws/src/ros-visualization
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && cd turtlebot3_simulations && git checkout ros2
WORKDIR /opt/ros2_ws
RUN colcon build --packages-up-to turtlebot3_gazebo turtlebot3_description turtlebot3_bringup --packages-skip-build-finished


ENV GAZEBO_MODEL_PATH /opt/ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models
ENV TURTLEBOT3_MODEL waffle


WORKDIR /toolkit
RUN cd /toolkit/ && \
	wget -q https://download-cf.jetbrains.com/python/pycharm-community-2019.2.3.tar.gz && \
	tar -xzf pycharm-community-2019.2.3.tar.gz && \
    rm -f /toolkit/pycharm-community-2019.2.3.tar.gz /toolkit/redis-3.2.9.tar.gz

RUN sed -i '1isource /opt/ros2_ws/install/setup.bash;' /root/.bashrc && \
    sed -i '1iexport GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models;' /root/.bashrc && \
    sed -i '1iexport TURTLEBOT3_MODEL=waffle' /root/.bashrc && \
    sed -i '1iexport AMENT_PREFIX_PATH=/opt/ros2_ws/install/costmap_converter:$AMENT_PREFIX_PATH' /root/.bashrc


# TEB local planner
RUN apt install -y libsuitesparse-dev

WORKDIR /opt/ros2_ws/src/overlay/src
RUN git clone https://github.com/rst-tu-dortmund/costmap_converter.git && cd costmap_converter && git checkout ros2
WORKDIR /opt/ros2_ws
RUN colcon build --packages-up-to tf2_eigen costmap_converter costmap_converter_msgs --packages-skip-build-finished

#g20
WORKDIR /opt/ros2_ws/src/overlay/src
RUN git clone https://github.com/RainerKuemmerle/g2o.git && cd g2o && cmake . && make -j4 && make install

RUN git clone https://github.com/logivations/teb_local_planner.git && cd teb_local_planner && git checkout eloquent_changes
RUN colcon build --packages-up-to teb_local_planner teb_msgs --packages-skip-build-finished --symlink-install

