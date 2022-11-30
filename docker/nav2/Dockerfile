FROM nvcr.io/nvidia/isaac/ros:aarch64-humble-nav2_6dfaf7adbe190f1181c3a0a2f2418760 as nav2



# Keep this at the end of Docker file
ARG UNAME=ros
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -u $UID -g $GID --system --create-home -o -s /bin/bash $UNAME
RUN usermod -aG sudo  $UNAME && echo "$UNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$UNAME
RUN chmod 0440 /etc/sudoers.d/$UNAME

USER $UNAME
ENV HOME=/home/ros


COPY ros_entrypoint.sh /
ENTRYPOINT [ "/ros_entrypoint.sh" ]
