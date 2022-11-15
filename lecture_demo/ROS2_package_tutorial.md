**PRE-REQUISITE** before dealing with ros2 packages:

* You will have to create a new ros2 workspace and create/add your new package/packages in there under `your_ws/src`. Creating a workspace is easy and you just have to follow the first steps of this (https://docs.ros.org/en/galactic/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) tutorial.

* Once you create a new package (https://docs.ros.org/en/galactic/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) you will also have to build it. Check again this tutorial to see how that is done. You will see that the building tool that we use for ros2 packages is colcon (a CMake-based build tool)

1. If you open the package above the first thing you will note is that on the root level of the package there are a couple of folders and 2 files. Let's firstly talk about those files: `CMakeLists.txt` and `package.xml`. Dunno if you guys have experience with make/cmake and build tools developed on top of those but, essentially, what the `CMakeLists.txt` is supposed to tell you at first glance is that the ros2 package is built using a cmake-based build tool. Now, as I briefly said above, there might be some confusion with this as we will be writing/launching python ros2 nodes and this is a package built with CMake (cpp - duh). Why we want to do this is because of a small issue with the pure python packages in the current ros2 version that we are using. So basically, we build them as cpp packages but write also python nodes in the context of them, and it works just fiiine. 


2. What you will find in the CMakeLists.txt is a list of build directives, telling the build tool (`colcon` in our case) what cpp version to use, enabling python support, what are the packages that the current package depends on, enabling testing, correctly defining and setting the new messages, services and actions defined in the context of the new package, etc. What you need to care about in here is how to add: new dependencies, messages, services or actions - add them to the corresponding section in the same way as shown in the demo package. 


3. In a way symbiotic to the `CMakeLists.txt` is the `package.xml`. It is also used as a means of configuring your ros2 package and basically what you need to know about it is that, on top of what it contains already, you should add any other subsequent package that your new package depends on within a new `<depend></depend>` tag.


4. In the `/action`, `/msg` and `/srv` folders you will find definitions for some basic example message, service and action files. If you have to create some of your own you can follow those examples and I am pretty sure there might be more around the internet. Make sure that once you create a new file of those types, you also add then in the `CMakeLists.txt` under the right section and rebuild the package with `colcon build --packages-select <package_name>`.


5. Over in the `/launch` folder you will find the python launch file that is doing the launching of the ros2 node we define in this package under the folder with the same name as the package. In the launch file you will see how launch arguments (which can be also sent form the command line), can be defined and then sent over to the node. In a launch file you can launch any number of nodes at the same time, just define them in the same way and append them to the `launch_list`. A ros2 node can be launched/ran in 2 ways...either through a launch file as seen in this demo package or by just running the node. Once the package is built you can launch the launch file with this command from the terminal: `ros2 launch lecture_demo demo_publisher.launch.py dummy_arg_1:=2.0 dummy_arg_2:=test` or more generic `ros2 launch <package_name> <launc_file.launch.py> <optional_list_of_launch_arguments_and_their_values>`. If you don't want to use a launch file you can also simply run `ros2 run lecture_demo demo_publisher.py` but in this case the python file where the node is defined needs to also be marked as executable. 


6. Lastly, in the folder having the same name as the package you can see the python file where the demo_publisher node is defined. This is a bare-bone node that publishes a pre-defined `Twist` message once every 0.05 seconds. In the constructor of the node you can see how launch arguments can be passed to the node and furtherly manipulated. In such a node you can set up publishers, subscribers for different kinds of topics and also define the logic for your autonomous exploration algorithm. To see what else you can do in the context of the node I advise using the official tutorials or if you don't find something, just ask us!