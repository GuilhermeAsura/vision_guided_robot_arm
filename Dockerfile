FROM osrf/ros:humble-desktop-full

ENV TERM=xterm-256color

# Copia o Webots que você baixou
COPY webots-R2025a-x86-64.tar.bz2 /tmp/webots.tar.bz2

# Instala ROS, Python e dependências de RUNTIME do Webots
RUN apt-get update && apt-get install -y \
    nano \
    vim \
    git \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros-gz \
    ros-humble-turtlesim \
    '~nros-humble-rqt*' \
    ros-humble-urdf-tutorial \
    # --- Dependências do Projeto ---
    python3-opencv \
    python3-numpy \
    ros-humble-webots-ros2 \
    # --- Dependências de Runtime do Webots (da Cyberbotics) ---
    wget \
    xvfb \
    locales \
    bzip2 \
    && rm -rf /var/lib/apt/lists/*

# Executa o script oficial de instalação de dependências do Webots
RUN wget -q https://raw.githubusercontent.com/cyberbotics/webots/master/scripts/install/linux_runtime_dependencies.sh \
    && chmod +x linux_runtime_dependencies.sh \
    && ./linux_runtime_dependencies.sh \
    && rm ./linux_runtime_dependencies.sh \
    && rm -rf /var/lib/apt/lists/*

# Extrai o Webots e limpa
RUN tar xjf /tmp/webots.tar.bz2 -C / \
    && mv /webots /usr/local/webots \
    && rm /tmp/webots.tar.bz2

# Configura Variáveis de Ambiente (da Cyberbotics)
ENV WEBOTS_HOME=/usr/local/webots
ENV PATH="/usr/local/webots:${PATH}"
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,compute,utility

# Configura locales (da Cyberbotics)
RUN locale-gen en_US.UTF-8
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'

# Cria o usuário não-root
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Configura o sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

RUN usermod -aG dialout ${USERNAME}

# Copia os scripts
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

# Configura o entrypoint
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash","-l"]