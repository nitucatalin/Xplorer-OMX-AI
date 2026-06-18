# ============================================================================
# Dockerfile — mediu de simulare Gazebo pentru scenariul de orchestrare
# Xplorer-A + OMX-AI (licență Nițu George-Cătălin)
# ----------------------------------------------------------------------------
# Bază: tiryoh/ros2-desktop-vnc:jazzy
#   = Ubuntu 24.04 + ROS 2 Jazzy desktop + desktop XFCE accesibil din browser
#     (noVNC pe portul 80 din container). Imagine multi-arch (amd64 + arm64),
#     deci merge și pe Mac cu Apple Silicon.
# ============================================================================
FROM tiryoh/ros2-desktop-vnc:jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-rviz-plugins \
    ros-jazzy-ros-gz \
    ros-jazzy-xacro \
    python3-colcon-common-extensions \
    python3-yaml \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Rendering software (fără GPU în container)
ENV LIBGL_ALWAYS_SOFTWARE=1

# Sursare automată în orice terminal nou din desktopul web
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /home/ubuntu/.bashrc && \
    echo '[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash' >> /home/ubuntu/.bashrc
