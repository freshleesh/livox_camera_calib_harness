FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      libceres-dev \
      libeigen3-dev \
      ros-noetic-cv-bridge \
      ros-noetic-pcl-conversions \
      ros-noetic-pcl-ros \
      git \
      build-essential \
      cmake \
      python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
 && echo "[ -f /catkin_ws/devel/setup.bash ] && source /catkin_ws/devel/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
