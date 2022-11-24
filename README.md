# XploreR

## Requirements

To run the project as it was intended, the following is required:
- Docker (including docker-compose): recommended to install Docker Desktop
- (Windows) VcXsrv Windows X Server: https://sourceforge.net/projects/vcxsrv/


## First-time setup

For `turtlesim` and similar GUI ROS packages to work with Docker, the following steps are necessary. Follow whichever fits your OS.

### Windows

1. Download and install VcXsrv Windows X Server: 
https://sourceforge.net/projects/vcxsrv/

2. Start XLaunch (VcXsrv Windows X Server). **Note:** this can probably be replaced with other X server program.
    - Press next until you get to `Extra settings` tab.
    - Deselect `Native opengl`.
    - Select `Disable access control`.
    - **Note:** sometimes when running simulations XLaunch might get buggy, so you have to kill the whole process and start it again.

3. Get your local IP from `ipconfig` (eg. 192.168.100.2).

4. Create and open `environment.env` file. Copy this template and edit `IP` (your local IP from ipconfig) and `WORKDIR` (your working directory, i.e. where these files are). **Note:** `WORKDIR` might be unnecessary.

```txt
IP=192.168.229.233
WORSKSPACES_DIR=C:/Users/fabia/Documents/_imlex/uef/RXR/xplorer
DISPLAY=${IP}:0.0

ROS_DOMAIN_ID=21
TURTLEBOT3_MODEL=waffle

COMPOSE_DOCKER_CLI_BUILD=0
```

For example: `IP=192.168.229.269` and `WORSKSPACES_DIR=C:/Users/fabia/Documents/_imlex/uef/RXR/xplorer`.

### Linux

To run Docker without `sudo`:

```bash
sudo groupadd docker
sudo gpasswd -a $USER docker
newgrp docker
```

Run the following command:

```bash
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' turtlesim`
```

**IMPORTANT:** This command is required on every reboot.

Next, create the `environment.env` file in this directory with the following contents:

```txt
DISPLAY=:0

ROS_DISTRO=galactic
ROS_DOMAIN_ID=1
TURTLEBOT3_MODEL=waffle

COMPOSE_DOCKER_CLI_BUILD=0
```

To check if the `DISPLAY` environment variable is correct, open the terminal and run

```bash
echo $DISPLAY
```

Write the result to `DISPLAY` in `environment.env` (normally, it's either `:0` or `:1`). Now you're ready to run the project!

**Note:** If the project still doesn't work, you might need to install [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#setting-up-nvidia-container-toolkit), then run the following:

```bash
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

## Running turtlesim

If on **Windows**, open XLaunch (X server) and configure it like previously. If on **Linux**, remember to run 
```bash
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' turtlesim`
```
on every reboot.

To run a `turtlesim` simulation, run the following command:

```bash
docker-compose up
```

You should see something like:

```bash
turtlesim    | [INFO] [1667985097.122394400] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]
```

To get `turtlesim turtle_teleop_key` to work, open a new terminal and `exec` into the `turtlesim` container:

```bash
docker exec -it turtlesim bash
```

Run the teleop package with:

```bash
ros2 run turtlesim turtle_teleop_key
```

You should see something like this:

```bash
Reading from keyboard
---------------------------
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
'Q' to quit.
```

**IMPORTANT:** if you get the `ros2 not found` error, ROS2 wasn't sourced automatically. To fix this, you have to run the following:

```bash
. ros_entrypoint.sh
```

## Rosbridge Server

Source: https://foxglove.dev/blog/using-rosbridge-with-ros2

The rosbridge server node is launched with the `docker-compose up` command. It'll create a websocket on `localhost:9090`. To test if it's working, open `rosbridge-test.html` and check if the connection is *successful*. Then, publish a message to the ros topic `/my_topic` with:

```bash
ros2 topic pub /my_topic std_msgs/String "data: Hello world!" -1
```

The message should show up in the html page.