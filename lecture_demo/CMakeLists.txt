cmake_minimum_required(VERSION 3.8)
project(lecture_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# here add any other ros2 packages (either defualt packages that come with the distro or packages made by you in the same workspace) that the current package depends on
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# msgs, services and actions dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/ExampleMessage.msg"
)

set(srv_files
  "srv/ExampleService.srv"
)

set(action_files
  "action/ExampleAction.action"
)

set(dependencies
    builtin_interfaces
    std_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  ${action_files}
  DEPENDENCIES ${dependencies}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  lecture_demo/demo_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# this is needed if your package also has written tests (unit, integration, etc.) written for some of the nodes, and placed in a "test" subfolder
if(BUILD_TESTING)
    add_subdirectory(test)
endif()

ament_package()
