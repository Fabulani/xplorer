FROM osrf/ros:galactic-desktop

# ROS distribution
ENV ROS_DISTRO=galactic

# Update apt so that new packages can be installed properly
RUN apt-get update && apt-get install -y wget \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*rm 

# Install ROS2-galactic packages
RUN sudo apt-get install ros-galactic-rosbridge-suite -y \
    && sudo apt-get install ros-galactic-turtlebot3* -y \
    && sudo apt-get install ros-galactic-navigation2 -y \
    && sudo apt-get install ros-galactic-nav2-bringup -y

# Install gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
    && apt-get update -y && apt-get install -y gazebo11 libgazebo11-dev

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
RUN echo '. /opt/ros/$ROS_DISTRO/setup.sh' >> ~/.bashrc && \
    echo '. /xplorer_ws/install/setup.bash' >> ~/.bashrc

# Required in Linux systems. Gives proper permissions to entrypoint file.
RUN ["chmod", "+x", "/ros_entrypoint.sh"]
