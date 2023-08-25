cmake_minimum_required(VERSION 3.10.2)
project(uav_wrapper_py)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS std_msgs)
catkin_install_python(PROGRAMS src/controller.py src/example.py src/tf_publisher.py src/uav.py src/wall.py src/waypoint_drop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
