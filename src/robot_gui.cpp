#define CVUI_IMPLEMENTATION
#include "ros/init.h"
#include "ros/node_handle.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>

#define WINDOW_NAME "ROBOT CVUI"

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui_node");
  ros::NodeHandle nh;

  cv::Mat frame = cv::Mat(200, 500, CV_8UC3);
  cvui::init(WINDOW_NAME);
  while (true) {
    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    // Render UI components to the frame
    cvui::text(frame, 110, 80, "Hello, world!");
    cvui::text(frame, 110, 120, "cvui is awesome!");

    // Update cvui stuff and show everything on the screen
    cvui::imshow(WINDOW_NAME, frame);

    if (cv::waitKey(20) == 27) {
      break;
    }
  }
  return 0;
}