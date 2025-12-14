FROM osrf/ros:humble-desktop-full

ENV TERM=xterm-256color

# Copy the downloaded Webots package
COPY webots-R2025a-x86-64.tar.bz2 /tmp/webots.tar.bz2

# Install ROS, Python, and Webots RUNTIME dependencies
RUN apt-get update && apt-get install -y \
    nano \
    vim \
    git \
    xterm \
    python3-pip \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros-gz \
    ros-humble-turtlesim \
    '~nros-humble-rqt*' \
    ros-humble-urdf-tutorial \
    # Project Dependencies
    python3-opencv \
    python3-numpy \
    ros-humble-webots-ros2 \
    # Webots Runtime Dependencies (Cyberbotics)
    wget \
    xvfb \
    locales \
    bzip2 \
    && rm -rf /var/lib/apt/lists/*

# Install IKPY library via pip
RUN pip3 install ikpy

# Execute official Webots dependency installation script
RUN wget -q https://raw.githubusercontent.com/cyberbotics/webots/master/scripts/install/linux_runtime_dependencies.sh \
    && chmod +x linux_runtime_dependencies.sh \
    && ./linux_runtime_dependencies.sh \
    && rm ./linux_runtime_dependencies.sh \
    && rm -rf /var/lib/apt/lists/*

# Extract Webots and clean up
RUN tar xjf /tmp/webots.tar.bz2 -C / \
    && mv /webots /usr/local/webots \
    && rm /tmp/webots.tar.bz2

# Configure Environment Variables (Cyberbotics)
ENV WEBOTS_HOME=/usr/local/webots
ENV PATH="/usr/local/webots:${PATH}"
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility

# Configure locales
RUN locale-gen en_US.UTF-8
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'

# Create non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Configure sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

RUN usermod -aG dialout ${USERNAME}

# Copy scripts
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

# Set up entrypoint
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash","-l"]