FROM osrf/ros:galactic-desktop

ENV ROS_DISTRO=galactic

WORKDIR /

RUN apt-get update && apt-get install -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*rm 

RUN sudo apt-get install ros-galactic-rosbridge-suite -y

# Replace ros_entrypoint.sh
COPY ros_entrypoint.sh /ros_entrypoint.sh

# Sources ROS on every new terminal automatically
# The YOUR_WORKSPACE_PATH is the absolute path to the workspace.
#? How to set it automatically in the Dockerfile?
# Add this for every new line: && \
# echo '. <YOUR_WORKSPACE_PATH>/install/setup.bash' >> ~/.bashrc`
RUN echo '. /opt/ros/$ROS_DISTRO/setup.sh' >> ~/.bashrc

RUN ["chmod", "+x", "/ros_entrypoint.sh"]