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

DOC NVIDIA STUFF

Run the following command:

```bash
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' turtlesim`
```

**NOTE: replace ros_entrypoint.sh to include the bash commands to automatically do this on new terminals. Also, set env variables in the docker-compose file.** 

## Running turtlesim

To run a `turltesim` simulation, run the following command:

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

Source ROS with (don't forget the dot!):

```bash
. ros_entrypoint.sh
```

Finally, run the teleop package with:

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

