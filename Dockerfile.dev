FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    tmux \
    python3-pip \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    && rm -rf /var/lib/apt/lists/*

# robot_web_bridge web UI/API deps (FastAPI + HTMX/Jinja stack)
RUN pip install --no-cache-dir \
    fastapi "uvicorn[standard]" jinja2 pyyaml python-multipart

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["bash"]
