FROM osrf/ros:galactic-desktop

# ROS distribution
ENV ROS_DISTRO=galactic

# Update apt so that new packages can be installed properly. wget for gazebo, dos2unix for line endings fix
RUN apt-get update && apt-get install -y wget dos2unix \
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
#TODO: instead of COPY, clone https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main/tutorials/ros_unity_integration/ros2_packages
#! This might not be necessary, since we have the navigation package for Unity
RUN mkdir /xplorer_ws/src -p
COPY xplorer_ws/src xplorer_ws/src
RUN cd /xplorer_ws && \
    git clone https://github.com/shani1610/m-explore-ros2.git src/m-explore-ros2 --branch main && \
    git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git src/ROS-TCP-Endpoint --branch main-ros2 && \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    rosdep install --from-paths . --ignore-src -r -y && \
    colcon build

# Replace ros_entrypoint.sh
COPY ros_entrypoint.sh /ros_entrypoint.sh

# Sources ROS on every new terminal automatically
RUN echo '. /opt/ros/$ROS_DISTRO/setup.sh' >> ~/.bashrc && \
    echo '. /xplorer_ws/install/setup.bash' >> ~/.bashrc

# Use dos2unix to convert the line endings, remove dos2unix and wget, then clean up files created by apt-get
RUN dos2unix /ros_entrypoint.sh && apt-get --purge remove -y dos2unix wget && rm -rf /var/lib/apt/lists/*

# Required in Linux systems. Gives proper permissions to entrypoint file.
RUN ["chmod", "+x", "/ros_entrypoint.sh"]
