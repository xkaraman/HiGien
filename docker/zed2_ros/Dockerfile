# FROM stereolabs/zed:3.8-tools-devel-jetson-jp5.0.2 as zed2_sdk
FROM dustynv/ros:humble-desktop-l4t-r35.1.0 as ros2_humble

# After using an arg in a `FROM` line, the arg is lo
ARG L4T_MAJOR=35
ARG L4T_MINOR=1
ARG ZED_SDK_MAJOR=3
ARG ZED_SDK_MINOR=8

#This environment variable is needed to use the streaming features on Jetson inside a container
ENV LOGNAME root
ENV DEBIAN_FRONTEND noninteractive

#Install dependencies
RUN apt-get update -y && apt-get install -y sudo apt-utils apt-transport-https lsb-release udev usbutils git 

#Install ZED SDK
RUN apt-get update -y && apt-get install -y --no-install-recommends wget less cmake curl gnupg2 \
  build-essential python3 python3-pip python3-dev python3-setuptools libusb-1.0-0-dev zstd -y && \
  sudo pip install protobuf && \
  echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release ; \
  wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/l4t${L4T_MAJOR}.${L4T_MINOR}/jetsons && \
  chmod +x ZED_SDK_Linux_JP.run ; ./ZED_SDK_Linux_JP.run silent skip_tools && \
  rm -rf /usr/local/zed/resources/* && \
  rm -rf ZED_SDK_Linux_JP.run && \
  rm -rf /var/lib/apt/lists/*

#This symbolic link is needed to use the streaming features on Jetson inside a container
RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so


# Install zed_ros2_wrapper dependencies
# diagnostic_updater
WORKDIR /ros2_workspaces/src/
RUN git clone https://github.com/ros/diagnostics.git && cd diagnostics && git checkout ros2-devel
# xacro
WORKDIR /ros2_workspaces/src/
RUN git clone https://github.com/ros/xacro.git && cd xacro && git checkout ros2

WORKDIR /ros2_workspaces/src/
RUN git clone  --recursive https://github.com/stereolabs/zed-ros2-wrapper.git

WORKDIR /ros2_workspaces/
RUN /bin/bash -c "source /opt/ros/humble/install/setup.bash; rosdep install --from-paths src --ignore-src -r -y"

WORKDIR /ros2_workspaces/
RUN /bin/bash -c "source /opt/ros/humble/install/setup.bash;colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release" 

ENV ROS_OVERLAY="/ros2_workspaces"

# setup bashrc
# RUN cp /etc/skel/.bashrc ~/
RUN echo 'source ${ROS_OVERLAY}/install/setup.bash' >> /root/.bashrc
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY ./ros_entrypoint.sh  /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

STOPSIGNAL SIGINT

# # setup entrypoint
# COPY ./ros_entrypoint.sh 

# ENTRYPOINT ["/ros_entrypoint.sh"]
# CMD ["bash"]