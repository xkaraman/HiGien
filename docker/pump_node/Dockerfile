ARG ROS_DISTRO=humble

FROM dustynv/ros:humble-desktop-l4t-r35.1.0 as ros2_humble

ARG UNAME=ros
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -u $UID -g $GID --system --create-home -o -s /bin/bash $UNAME
RUN usermod -aG sudo  $UNAME && echo "$UNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$UNAME
RUN chmod 0440 /etc/sudoers.d/$UNAME
RUN newgrp dialout
RUN usermod -aG dialout ros

USER $UNAME
ENV HOME=/home/ros
WORKDIR $HOME/ros2_ws
# select bash as default shell
SHELL ["/bin/bash", "-c"]
RUN pip3 install pyserial
# # install everything needed
# RUN git clone https://github.com/xkaraman/higien.git src/higien -b main

# RUN source /opt/ros/$ROS_DISTRO/install/setup.bash && \
#   rosdep update && \
#   rosdep install --from-paths src --ignore-src -y && \
#   colcon build --symlink-install

## SETUP user to access and group
# RUN sudo cp src/sllidar_ros2/scripts/rplidar.rules /etc/udev/rules.d/
# RUN sudo newgrp dialout
# RUN sudo usermod -aG dialout ros

RUN echo "source /opt/ros/$ROS_DISTRO/install/setup.bash" >> ~/.bashrc
# RUN echo "source $HOME/ros2_ws/install/local_setup.bash" >> ~/.bashrc

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY ros_entrypoint.sh /

# Without this line LIDAR doesn't stop spinning on container shutdown. Default is SIGTERM. 
# STOPSIGNAL SIGINT