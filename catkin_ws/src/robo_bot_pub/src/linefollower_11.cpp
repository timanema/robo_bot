#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <thread>
#include <chrono>
#include <iostream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "linefollower");

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  geometry_msgs::Twist msg;

  while (ros::ok()) {
      msg.angular.x = 90;

      pub.publish(msg);
      ros::spinOnce();
      std::cout << "ping" << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
  }
}
