#pragma once

#include "ros/subscriber.h"
#include <string>
#define CVUI_IMPLEMENTATION

// #include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/node_handle.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

class ROBOT_CVUI {
public:
  ROBOT_CVUI();
  void run();
  void
  RobotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);

private:
  std::string robot_info_msg[10];

  ros::Publisher pub_;
  ros::Subscriber sub_;
  const std::string WINDOW_NAME = "ROBOT CVUI";
};