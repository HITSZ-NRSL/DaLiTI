FROM kalibr:noetic-20.04

ENV DEBIAN_FRONTEND=noninteractive

# Core build and runtime deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    curl \
    zsh \
    python3-pip \
    python3-catkin-tools \
    libeigen3-dev \
    libboost-all-dev \
    libopencv-dev \
    libpcl-dev \
    libceres-dev \
    libtbb-dev \
    libsuitesparse-dev \
    libx11-dev \
    libv4l-dev \
    libgd-dev \
    liblapack-dev \
    ros-noetic-cv-bridge \
    ros-noetic-message-generation \
    ros-noetic-message-runtime \
    ros-noetic-geometry-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-tf \
    ros-noetic-tf-conversions \
    ros-noetic-image-transport \
    ros-noetic-nodelet \
    ros-noetic-gtsam \
    ros-noetic-rosbag \
    && rm -rf /var/lib/apt/lists/*

# Install TooN headers (not packaged on focal)
RUN if [ ! -f /usr/local/include/TooN/TooN.h ] && [ ! -f /usr/include/TooN/TooN.h ]; then \
      mkdir -p /tmp/toon && cd /tmp/toon \
      && (curl -fL --retry 5 --retry-delay 2 -o toon.tar.gz https://github.com/edrosten/TooN/archive/refs/heads/master.tar.gz \
          || curl -fL --retry 5 --retry-delay 2 -o toon.tar.gz https://gitlab.com/edrosten/TooN/-/archive/master/TooN-master.tar.gz) \
      && tar -xzf toon.tar.gz --strip-components=1 \
      && if [ -d include/TooN ]; then cp -r include/TooN /usr/local/include/TooN; \
         elif [ -d TooN ]; then cp -r TooN /usr/local/include/TooN; \
         else echo \"TooN headers not found in archive\" && exit 1; fi \
      && rm -rf /tmp/toon; \
    fi

# Setup workspace
WORKDIR /ws
RUN mkdir -p /ws/src
COPY src/DaLiTI /ws/src/DaLiTI

# Build with catkin tools
RUN /bin/zsh -c "source /opt/ros/noetic/setup.zsh && cd /ws && catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release && catkin build"

# Change root user's default shell to zsh
RUN chsh -s /usr/bin/zsh root

# Setup zsh environment for ROS
RUN echo 'source /opt/ros/noetic/setup.zsh' >> /root/.zshrc && \
    echo 'source /ws/devel/setup.zsh 2>/dev/null || true' >> /root/.zshrc && \
    echo 'cd /ws' >> /root/.zshrc

# Default to zsh
ENV SHELL=/usr/bin/zsh
CMD ["/usr/bin/zsh"]
