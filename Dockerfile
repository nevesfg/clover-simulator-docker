FROM osrf/ros:noetic-desktop-full

# Instalar dependências necessárias, incluindo utilitários gráficos e bibliotecas OpenGL
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    python3-rosinstall-generator \
    python3-vcstool \
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-cv-camera \
    ros-noetic-web-video-server \
    ros-noetic-rosbridge-suite \
    ros-noetic-joy \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-tf2-web-republisher \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-cmake-modules \
    libgeographic-dev \
    geographiclib-tools \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# Configurar o workspace do Clover
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

# Comentei mas nao ê necessario descomentar, 
# vc vai clonar o clover seguindo os passos da doc dele
# RUN git clone --depth 1 https://github.com/CopterExpress/clover

# Instalar dependências do projeto com rosdep
WORKDIR /root/catkin_ws
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*

# Compilar o workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Fazer backup do workspace compilado para um diretório interno
RUN cp -r /root/catkin_ws /root/catkin_ws_default

# Configurar o ambiente de execução
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Copiar o entrypoint script para a imagem
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Definir diretório de trabalho padrão
WORKDIR /root/catkin_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
