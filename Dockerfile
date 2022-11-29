FROM osrf/ros:galactic-desktop

ENV ROS_DISTRO=galactic

# Update apt so that new packages can be installed properly
RUN apt-get update && apt-get install -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*rm 

# Install rosbridge
RUN sudo apt-get install ros-galactic-rosbridge-suite -y

# Copy and build ROS2 packages inside the workspace
RUN mkdir /xplorer_ws/src -p && \
    cd /xplorer_ws && \
    git clone https://github.com/robo-friends/m-explore-ros2.git src/m-explore-ros2 --branch main && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install --from-paths . --ignore-src -r -y && \
    colcon build

# Replace ros_entrypoint.sh
COPY ros_entrypoint.sh /ros_entrypoint.sh

# Sources ROS on every new terminal automatically
# The YOUR_WORKSPACE_PATH is the absolute path to the workspace.
#? How to set it automatically in the Dockerfile?
# Add this for every new line: && \
# echo '. <YOUR_WORKSPACE_PATH>/install/setup.bash' >> ~/.bashrc
RUN echo '. /opt/ros/$ROS_DISTRO/setup.sh' >> ~/.bashrc && \
    echo '. /xplorer_ws/install/setup.bash' >> ~/.bashrc

RUN ["chmod", "+x", "/ros_entrypoint.sh"]
