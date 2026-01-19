FROM ros:humble-ros-base

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-colcon-common-extensions \
    python3-serial rsync openssh-client \
    ros-humble-cv-bridge ros-humble-image-transport \
    ros-humble-v4l2-camera \
    && rm -rf /var/lib/apt/lists/*

# OpenCV python (for jpeg writes)
RUN pip3 install --no-cache-dir opencv-python

WORKDIR /work
