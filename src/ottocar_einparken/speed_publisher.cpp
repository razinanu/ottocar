#include "ros/ros.h"
#include "std_msgs/String.h"
#include "einparken_test/Angle.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"

#include <sstream>

void simulateParallelParking(std_msgs::Int16 &angle, std_msgs::Int8 &speed,
		ros::Time &timeSinceLastChange, int &counter)
{
	angle.data = 330;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "speed_publisher");
	ros::NodeHandle n;

	ros::Publisher angle_pub = n.advertise<std_msgs::Int16>("angle_cmd", 1);
	ros::Publisher speed_pub = n.advertise<std_msgs::Int8>("speed_cmd", 1);

	ros::Rate loop_rate(10);

	int counter = 0;
	ros::Time start = ros::Time::now();
	ros::Time timeSinceLastChange = ros::Time::now();
	std_msgs::Int16 angle;
	std_msgs::Int8 speed;

	while (ros::ok())
	{
//	  if (count <6)
//	  {
//		  angle.data = 378;
//		  speed.data = -8;
//	  }
//	  else if (count >= 12 && count < 20)
//	  {
//		  angle.data = 379;
//		  speed.data = 0;
//	  }
//	  else if (count >= 20 && count < 24)
//	  {
//		  angle.data = 445;
//		  speed.data = 8;
//	  }
//	  else if (count >= 24 && count < 26)
//	  {
//		  angle.data = 379;
//		  speed.data = 10;
//	  }
//	  else if (count >= 26 && count < 28)
//	  {
//		  angle.data = 305;
//		  speed.data = 8;
//	  }
//	  else
//	  {
//		  angle.data = 379;
//		  speed.data = 0;
//	  }

		simulateParallelParking(angle, speed, timeSinceLastChange, counter);

		ROS_INFO("%d, %d", angle.data, speed.data);
		angle_pub.publish(angle);
		speed_pub.publish(speed);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
