#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const geometry_msgs::Twist msg)
{
  ROS_INFO("Linear: %f %f %f", msg.linear.x, msg.linear.y, msg.linear.z);
  ROS_INFO("Angular: %f %f %f", msg.angular.x, msg.angular.y, msg.angular.z);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "debug");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, chatterCallback);

  ros::spin();

  return 0;
}
