#pragma once

#include "ros/service_client.h"
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
  void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);

private:
  std::string robot_info_msg[10];
  nav_msgs::Odometry odom_msg;

  ros::Publisher cmd_vel_pub;
  ros::Subscriber robot_info_sub;
  ros::Subscriber odom_sub;
  ros::ServiceClient distance_tracker_service_client, reset_service_client;
  const std::string WINDOW_NAME = "ROBOT CVUI";
};