#include "robot_gui/robot_info_gui.h"

#include <ros/ros.h>
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/node_handle.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

ROBOT_CVUI::ROBOT_CVUI() {
  ros::NodeHandle nh;
  robot_info_sub = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(
      "/robot_info", 2, &ROBOT_CVUI::RobotInfoCallback, this);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

void ROBOT_CVUI::RobotInfoCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  robot_info_msg[0] = msg->data_field_01;
  robot_info_msg[1] = msg->data_field_02;
  robot_info_msg[2] = msg->data_field_03;
  robot_info_msg[3] = msg->data_field_04;
  robot_info_msg[4] = msg->data_field_05;
  robot_info_msg[5] = msg->data_field_06;
  robot_info_msg[6] = msg->data_field_07;
  robot_info_msg[7] = msg->data_field_08;
  robot_info_msg[8] = msg->data_field_09;
  robot_info_msg[9] = msg->data_field_10;
}

void ROBOT_CVUI::run() {
  cv::Mat frame = cv::Mat(750, 320, CV_8UC3);
  geometry_msgs::Twist cmd_vel_msg;
  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    frame = cv::Scalar(49, 52, 49);
    // General Info Area
    cvui::window(frame, 10, 10, 300, 150, "Info");
    for (int i = 0; i <= 8; i++) {
      cvui::printf(frame, 12, i * 15 + 35, 0.4, 0xffffff, "%s",
                   robot_info_msg[i].c_str());
    }
    // Teleoperation Buttons
    if (cvui::button(frame, 110, 170, 100, 50, "Forward")) {
      cmd_vel_msg.linear.x += 0.1;
    }
    if (cvui::button(frame, 5, 225, 100, 50, "Left")) {
      cmd_vel_msg.angular.z += 0.1;
    }
    if (cvui::button(frame, 110, 225, 100, 50, "Stop")) {
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = 0.0;
    }
    if (cvui::button(frame, 215, 225, 100, 50, "Right")) {
      cmd_vel_msg.angular.z -= 0.1;
    }
    if (cvui::button(frame, 110, 280, 100, 50, "Backward")) {
      cmd_vel_msg.linear.x -= 0.1;
    }
    cmd_vel_pub.publish(cmd_vel_msg);
    // Current velocity
    cvui::window(frame, 10, 340, 145, 40, "Linear velocity:");
    cvui::printf(frame, 12, 375, 0.4, 0xff0000, "%.2f m/sec",
                 cmd_vel_msg.linear.x);

    cvui::window(frame, 165, 340, 145, 40, "Angular velocity:");
    cvui::printf(frame, 167, 375, 0.4, 0xff0000, "%.2f rad/sec",
                 cmd_vel_msg.angular.z);
    // Robot position

    // Update
    cvui::update();

    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }

    ros::spinOnce();
  }
}
