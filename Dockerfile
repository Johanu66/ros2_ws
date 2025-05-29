FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-vcstool \
    build-essential \
    git \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    x11-apps \
    libomp-dev \
    libqt5gui5 \
    libcanberra-gtk-module \
    ros-jazzy-desktop \
    && rm -rf /var/lib/apt/lists/*

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

# Install ROS packages via apt
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    ros-jazzy-launch-param-builder \
    ros-jazzy-turtlesim \
    ros-jazzy-example-interfaces \
    ros-jazzy-ompl \
    ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-ros2cli-common-extensions \
    ros-jazzy-joy \
    ros-jazzy-ament-lint-cmake \
    ros-jazzy-ament-cmake-copyright \
    ros-jazzy-ament-cmake-lint-cmake \
    ros-jazzy-stomp \
    ros-jazzy-ament-cmake-xmllint \
    libgz-plugin2-dev \
    libgz-utils3-dev \
    libgz-utils3 \
    libgz-tools2-dev \
    libgz-cmake4-dev \
    ros-jazzy-controller-manager-msgs \
    ros-jazzy-ros-testing \
    ros-jazzy-octomap-msgs \
    ros-jazzy-rsl \
    ros-jazzy-srdfdom \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-eigen-stl-containers \
    liboctomap-dev \
    ros-jazzy-gz-sim-vendor \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-warehouse-ros \
    ros-jazzy-geometric-shapes \
    ros-jazzy-ament-clang-format \
    ros-jazzy-gripper-controllers \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-picknik-reset-fault-controller \
    ros-jazzy-object-recognition-msgs \
    ros-jazzy-py-binding-tools \
    ros-jazzy-launch-pytest \
    ros-jazzy-position-controllers \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-picknik-twist-controller \
    ros-jazzy-ackermann-steering-controller \
    ros-jazzy-actuator-msgs \
    ros-jazzy-graph-msgs \
    ros-jazzy-gps-msgs \
    ros-jazzy-warehouse-ros-sqlite \
    ros-jazzy-vision-msgs \
    ros-jazzy-topic-tools \
    ros-jazzy-rmf-utils \
    libcli11-dev \
    ros-jazzy-ros2-control \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-ros2-control-cmake \
    ros-jazzy-effort-controllers \
    ros-jazzy-ament-cmake-google-benchmark \
    jupyter-notebook \
    ros-jazzy-ros2-controllers \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-ament-clang-tidy \
    ros-jazzy-osqp-vendor \
    ros-jazzy-ament-cmake-vendor-package \
    ros-jazzy-rviz-imu-plugin \
    ros-jazzy-rviz-imu-plugin \
    ros-jazzy-ruckig \
    xvfb \
    libgz-utils3-cli-dev \
    && rm -rf /var/lib/apt/lists/*

# Workspace setup
RUN mkdir -p /root/ros2_ws
WORKDIR /root/ros2_ws
# RUN git clone https://github.com/Johanu66/ros2_ws.git
COPY src src

# Clone repositories
COPY requirements_ros2.repos .
RUN vcs import --recursive src < requirements_ros2.repos


# Install Python dependencies
RUN pip3 install --no-cache-dir --break-system-packages \
    setuptools \
    colcon-common-extensions \
    colcon-cmake \
    colcon-ros-bundle \
    colcon-ros \
    rosdep

# Copier le fichier rosdep personnalisé dans le conteneur
COPY rosdep.yaml /etc/ros/rosdep/custom.yaml

# Ajouter ce fichier aux sources de rosdep
RUN echo "yaml file:///etc/ros/rosdep/custom.yaml" > /etc/ros/rosdep/sources.list.d/30-custom.list

# Mettre à jour la base rosdep avec cette nouvelle source
RUN rosdep update

# Install dependencies using rosdep
RUN . /opt/ros/jazzy/setup.sh && \
    rosdep install -y --from-paths src --ignore-src --rosdistro jazzy

# Build environment
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
