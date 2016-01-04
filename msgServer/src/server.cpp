#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <cstdio>
#include <sstream>

/**
 * Returns a Twist message.
 */
geometry_msgs::Twist getTwist(char c) {
	geometry_msgs::Twist msg;
	msg.linear.x = 1.0;
	return msg;
}

/**
 * This should send Twist messages according to what key you press.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "server");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("chatter", 1000);
  ros::Rate loop_rate(100);

  char c;
  
  system ("/bin/stty raw");
  while (ros::ok() && c != 3) {
    c = getchar();
    
    geometry_msgs::Twist msg = getTwist(c);

    ROS_INFO("Linear: %f %f %f", msg.linear.x, msg.linear.y, msg.linear.z);
    ROS_INFO("Angular: %f %f %f", msg.angular.x, msg.angular.y, msg.angular.z);

    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  system ("/bin/stty cooked");
  return 0;
}

