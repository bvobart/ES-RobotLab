#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <cstdio>
#include <sstream>

double speed = 5.0;

/**
 * Returns a Twist message according to the character pressed.
 *
 * The X in the linear component is positive for forwards,
 * and negative for backwards.
 *
 * The X in the angular component is positive for turning left,
 * and negative for turning right.
 *
 * Both are scaled according to the speed, which initializes to 5
 * and can be incremented or decremented using the x and z keys
 * respectively. r resets the speed.
 */
geometry_msgs::Twist getTwist(char c) {
	geometry_msgs::Twist msg;
	switch (tolower(c)) {
		case 'w': //forward
			msg.linear.x = speed;
			break;
		case 's': //backward
			msg.linear.x = -speed;
			break;
		case 'a': //left
			msg.angular.x = speed;
			break;
		case 'd': //right
			msg.angular.x = -speed;
			break;
		case 'z': //slow down
			if (speed > 0)
				speed -= 0.5;
			break;
		case 'x': //speed up
			if (speed < 10)
				speed += 0.5;
			break;
		case 'r': //reset speed
			speed = 5.0;
			break;
		default:
			ROS_INFO("Unknown key. Speed is %f", speed);
			break;
	}
	return msg;
}

/**
 * This sends Twist messages according to what key you press.
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

    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  system ("/bin/stty cooked");
  return 0;
}

