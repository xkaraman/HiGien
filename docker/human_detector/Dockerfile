ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r35.1.0

FROM $BASE_IMAGE as librealsense-builder
#################################
#   Librealsense Builder Stage  #
#################################
# To avoid waiting for input during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Builder dependencies installation
RUN apt-get update \
  && apt-get install -qq -y --no-install-recommends \
  build-essential \
  cmake \
  git \
  libssl-dev \
  libusb-1.0-0-dev \
  pkg-config \
  libgtk-3-dev \
  libglfw3-dev \
  libgl1-mesa-dev \
  libglu1-mesa-dev \    
  curl \
  python3 \
  python3-dev \
  ca-certificates \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/src
RUN git clone https://github.com/IntelRealSense/librealsense.git -b v2.51.1

# Build and install
RUN cd /usr/src/librealsense \
  && mkdir build && cd build \
  && cmake \
  -DFORCE_RSUSB_BACKEND:bool=true \
  -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
  -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
  -DCMAKE_INSTALL_PREFIX=/opt/librealsense/ \    
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_PYTHON_BINDINGS:bool=true \
  -DCMAKE_BUILD_TYPE=Release ../ \
  && make -j$(($(nproc)-1)) all \
  && make install 

######################################
#   librealsense on top of           #
#  ros2-humble-arm Base Image Stage  #
######################################
FROM dustynv/ros:humble-desktop-l4t-r35.1.0 as realsense2_ros2

# Copy binaries from builder stage
COPY --from=librealsense-builder /opt/librealsense/ /usr/local/
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib

# Install dep packages
RUN apt-get update \
  && apt-get install -y --no-install-recommends \	
  libusb-1.0-0 \
  udev \
  apt-transport-https \
  ca-certificates \
  curl \
  software-properties-common \
  && rm -rf /var/lib/apt/listsusr/local/*

## ROS2 workspace

# Shows a list of connected Realsense devices
# ENTRYPOINT ["/ros_entrypoint.sh"]
# CMD [ "rs-enumerate-devices", "--compact" ]


# ARG UNAME=ros
# ARG UID=1000
# ARG GID=1000
# RUN groupadd -g $GID -o $UNAME
# RUN useradd -m -u $UID -g $GID --system --create-home -o -s /bin/bash $UNAME
# RUN usermod -aG sudo  $UNAME && echo "$UNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$UNAME
# RUN chmod 0440 /etc/sudoers.d/$UNAME

# USER $UNAME
# ENV HOME=/home/ros
# WORKDIR $HOME/ros2_ws

# select bash as default shell
SHELL ["/bin/bash", "-c"]

WORKDIR /docker_ws

RUN git clone https://github.com/IntelRealSense/realsense-ros.git src/realsense-ros -b 4.51.1

# diagnostic_updater
# WORKDIR /ros2_workspaces/src/
RUN git clone https://github.com/ros/diagnostics.git src/diagnostics -b humble
# xacro
# WORKDIR /ros2_workspaces/src/
RUN git clone https://github.com/ros/xacro.git src/xacro -b ros2

RUN source /opt/ros/$ROS_DISTRO/install/setup.bash && \
  # rosdep update && \
  # rosdep install --from-paths src --ignore-src -y && \
  colcon build --symlink-install

# RUN sudo newgrp dialout
# RUN sudo usermod -aG plugdev ros
COPY --from=librealsense-builder /usr/lib/python3/dist-packages/pyrealsense2/ /usr/lib/python3/dist-packages/pyrealsense2/

COPY ros_entrypoint.sh /
# ENTRYPOINT [ "/ros_entrypoint.sh" ]


####################
# PYTORCH INCLUDED #
####################
FROM dustynv/ros:humble-pytorch-l4t-r35.1.0 as ros-pytorch-realsense

# RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
#   dpkg-reconfigure --frontend=noninteractive locales && \
#   update-locale LANG=en_US.UTF-8
# ENV LANG en_US.UTF-8

RUN pip3 install IPython tqdm matplotlib seaborn \
  gitpython scipy filterpy lap

# RUN export LD_PRELOAD=/usr/local/lib/python3.8/dist-packages/scikit_image.libs/libgomp-d22c30c5.so.1.0.0 
# RUN pip3 install scikit-image

# Install dep packages
RUN apt-get update \
  && apt-get install -y --no-install-recommends \	
  libusb-1.0-0 \
  udev \
  apt-transport-https \
  ca-certificates \
  curl \
  software-properties-common \
  && rm -rf /var/lib/apt/lists/*

# Copy binaries from builder stage
COPY --from=librealsense-builder /opt/librealsense /usr/local/
COPY --from=librealsense-builder /usr/lib/python3/dist-packages/pyrealsense2 /usr/lib/python3/dist-packages/pyrealsense2
COPY --from=librealsense-builder /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib


COPY --from=realsense2_ros2 /docker_ws /docker_ws
COPY ros_entrypoint.sh /
