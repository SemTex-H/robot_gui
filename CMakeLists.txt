cmake_minimum_required(VERSION 3.0.2)
project(robot_gui)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  robotinfo_msgs
  roscpp
  std_srvs
)

find_package(OpenCV 4 REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_gui
#  CATKIN_DEPENDS geometry_msgs nav_msgs robotinfo_msgs roscpp std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(robot_gui_node
src/robot_info_gui_node.cpp 
src/robot_info_gui.cpp)
target_link_libraries(robot_gui_node ${catkin_LIBRARIES} 
  ${OpenCV_LIBS})


# add_library(robot_gui src/robot_info_gui.cpp)
# add_executable(robot_gui_node src/robot_info_gui_node.cpp)
# target_link_libraries(robot_gui_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} robot_gui)