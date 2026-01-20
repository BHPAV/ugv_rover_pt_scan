FROM ros:humble-ros-base

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-colcon-common-extensions \
    python3-serial rsync openssh-client \
    ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-v4l2-camera \
    && rm -rf /var/lib/apt/lists/*

# OpenCV python (for jpeg writes)
# Pin numpy<2 for compatibility with ROS 2 Humble cv_bridge (compiled with numpy 1.x)
RUN pip3 install --no-cache-dir opencv-python "numpy<2"

WORKDIR /work
