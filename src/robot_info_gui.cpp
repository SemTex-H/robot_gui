#include "robot_gui/robot_info_gui.h"

#include <ros/ros.h>
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/node_handle.h"

ROBOT_CVUI::ROBOT_CVUI() {
  ros::NodeHandle nh;
  sub_ = nh.subscribe<robotinfo_msgs::RobotInfo10Fields>(
      "/robot_info", 2, &ROBOT_CVUI::RobotInfoCallback, this);
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
  int count = 0;
  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);
    // Create window at (220, 20) with size 250x80 (width x height) and title
    cvui::window(frame, 10, 10, 300, 130, "Info");

    // Show how many times the button has been clicked inside the window.
    for (int i = 1; i <= 8; i++) {
      cvui::printf(frame, 12, i * 15 + 20, 0.4, 0xffffff, "%s",
                   robot_info_msg[i].c_str());
    }
    // Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }

    ros::spinOnce();
  }
}
