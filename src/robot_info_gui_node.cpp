#include "robot_gui/robot_info_gui.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui_node");
  ROBOT_CVUI robot_cvui;
  robot_cvui.run();
  return 0;
}