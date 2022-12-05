# XploreR

- [XploreR](#xplorer)
- [Requirements](#requirements)
- [First-time setup](#first-time-setup)
  - [Windows](#windows)
  - [Linux](#linux)
- [Docker with GUIs](#docker-with-guis)
- [Running the project](#running-the-project)
  - [Detached mode](#detached-mode)
  - [Opening a new terminal inside a container](#opening-a-new-terminal-inside-a-container)
  - [Resume exploration](#resume-exploration)
- [Unity](#unity)


# Requirements

To fully run the project, the following is required:
- Docker (including docker-compose): recommended to install Docker Desktop
- (Windows, required for turtlebot3 gazebo simulation) VcXsrv Windows X Server: https://sourceforge.net/projects/vcxsrv/
- Unity 2020+


# First-time setup

For `turtlebot3_gazebo` and similar GUI ROS packages to work with Docker, the following steps are necessary. Follow whichever fits your OS.

## Windows

1. Download and install VcXsrv Windows X Server: 
https://sourceforge.net/projects/vcxsrv/

2. Start XLaunch (VcXsrv Windows X Server). **Note:** this can probably be replaced with other X server program.
    - Press next until you get to `Extra settings` tab.
    - Deselect `Native opengl`.
    - Select `Disable access control`.
    - **Note:** sometimes when running simulations XLaunch might get buggy, so you have to kill the whole process and start it again.

3. Get your local IP from `ipconfig`.
   - **Note:** this can be done by pressing the Windows key, typing `cmd`, selecting the `Command Prompt` app, then typing `ipconfig`. Search for a line like this:
    `IPv4 Address. . . . . . . . . . . : 10.143.144.69`

4. Open the `environment.env` file and paste your IP in the `DISPLAY` variable before `:0`, like so: `DISPLAY=10.143.144.69:0`. Your `environment.env` file should look like this:

```txt
DISPLAY=10.143.144.69:0

ROS_DISTRO=galactic
ROS_DOMAIN_ID=1
TURTLEBOT3_MODEL=waffle
GAZEBO_MODEL_PATH=/opt/ros/galactic/share/turtlebot3_gazebo/models

COMPOSE_DOCKER_CLI_BUILD=0
```

**IMPORTANT:** you'll need to update your IP every time it changes!

## Linux

To run Docker without `sudo`:

```bash
sudo groupadd docker
sudo gpasswd -a $USER docker
newgrp docker
```

Run the following command:

```bash
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' gazebo`
```

**IMPORTANT:** This command is required on every reboot.

In the `environment.env` file, check if `DISPLAY` is correct by opening a terminal and running:

```bash
echo $DISPLAY
```

Write the result to `DISPLAY` (normally, it's either `:0` or `:1`). Now you're ready to run the project!

**Note:** If the project still doesn't work, you might need to install [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit), then run the following:

```bash
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

# Docker with GUIs

If you've done the first-time setup, remember to do the following on every system reboot:

- If on **Windows**, open XLaunch (X server) and configure it like previously. Update your IP in the `environment.env` file.
- If on **Linux**, remember to run 
    ```bash
    xhost +local:`docker inspect --format='{{ .Config.Hostname }}' gazebo`
    ```
    on every reboot.

# Running the project

Spin-up the containers with:

```bash
docker-compose up
```

If this is your first time here, it might take a couple minutes to build the image. Once it's done, you should see `explore`, `gazebo`, and `rostcp` containers up and their messages.

To shutdown, use `CTRL+C` in the terminal running the containers.

## Detached mode

Alternatively, you can run the containers in detached mode:

```bash
docker-compose up -d
```

This will leave the terminal free while the containers run in the background. To shutdown, run the following:

```bash
docker-compose down
```

## Opening a new terminal inside a container

You can `docker exec` into any of the containers and run `ros2` commands from the get-go (sourcing is done automatically). For example, going into the `explore` container:

```bash
docker exec -it explore bash
```

## Resume exploration

Sometimes the `explore` node will stop exploration, reporting that there are no more frontiers. This can happen when the simulation takes too long to launch. To resume exploration, `exec` into a container and run the following command:

```bash
ros2 topic pub /explore/resume std_msgs/Bool '{data: true}' -1
```

This publishes a single message to the `/explore/resume` topic, toggling the exploration back on. If the exploration keeps stopping, remove the `-1` so it is constantly resumed.

#  Unity

This project is designed to communicate with a Unity scene running the `ROS-TCP Connector` and `Unity Robotics Visualizations` packages. This is achieved via the `rostcp` container running the `ROS-TCP-Endpoint` package from the `main-ros2` branch. All of this is based on the tutorials provided by Unity on Github:

- [Unity-Technologies/Unity-Robotics-Hub/tree/main/tutorials/ros_unity_integration](https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main/tutorials/ros_unity_integration)
- [Unity-Technologies/Robotics-Nav2-SLAM-Example](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example)

To replicate this, follow the `ros_unity_integration` tutorial first.

Furthermore, we used a VR scene to visualize data from the robot. The following headsets were tested:
- Samsung Odyssey with HMD Odyssey controllers (WMR)
- Pimax 8KX with both Index, Sword, and Vive controllers (SteamVR)

Made with Unity editor version `2021.3.14f1`.