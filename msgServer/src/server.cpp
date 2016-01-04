#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cstdio>
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "server");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(100);

  char c;
  
  system ("/bin/stty raw");
  while (ros::ok() && c != 3)
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    
    c = getchar();
	
    std::stringstream ss;
    ss << c;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  system ("/bin/stty cooked");
  return 0;
}
