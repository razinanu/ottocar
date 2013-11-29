#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "einparken_test/Angle.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"

#include <sstream>
#include <iostream>

enum state
{
	step1_forward,
	step2,
	step3,
	step4,
	step5,
	end

};

void drive(int speed_value, int angle_value, float time,
		state new_program_state, std_msgs::Int8 &speed, std_msgs::Int16 &angle,
		ros::Time &timeSinceLastChange, state &program_State)
{
	if (timeSinceLastChange > ros::Time::now() - ros::Duration(time))
	{
		speed.data = speed_value;
		angle.data = angle_value;
	}
	else
	{
		std::cout << "[ INFO] [-#-#-#-#-#.-#-#-#-#-]: beendet: "<< new_program_state << " :" << (ros::Time::now() - timeSinceLastChange) << std::endl;
		timeSinceLastChange = ros::Time::now();
		program_State = new_program_state;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "speed_publisher");
	ros::NodeHandle n;

	ros::Publisher angle_pub = n.advertise<std_msgs::Int16>("angle_cmd", 1);
	ros::Publisher speed_pub = n.advertise<std_msgs::Int8>("speed_cmd", 1);

	ros::Rate loop_rate(10);

	ros::Time start = ros::Time::now();		//not used yet
	ros::Time timeSinceLastChange = ros::Time::now();
	std_msgs::Int16 angle;
	std_msgs::Int8 speed;

	state program_State = step1_forward ;

	while (ros::ok())
	{
		switch (program_State)
		{
		//vorwaerts fahren, bis die Parkluecke erreicht wurde
		case (step1_forward):
		{
			drive(-20, 378, 1.0, step2, speed, angle, timeSinceLastChange, program_State);
			break;
		}
		//anhalten
		case (step2):
		{
			drive(0, 378, 1.0, step3, speed, angle, timeSinceLastChange, program_State);
			break;
		}
		//rueckwaerts fahren - rechts lenken
		case (step3):
		{
			drive(8, 430, 1.5, step5, speed, angle, timeSinceLastChange, program_State);
			break;
		}
		//anhalten
		case (step4):
		{
			drive(0, 378, 1.0, step5, speed, angle, timeSinceLastChange, program_State);
			break;
		}
		//rueckwaerts fahren - links lenken
		case (step5):
		{
			drive(8, 310, 1.0, end, speed, angle, timeSinceLastChange, program_State);
			break;
		}
		default:
		{
			drive(0, 378, 1.0, end, speed, angle, timeSinceLastChange, program_State);
			break;
		}
		}

		ROS_INFO("%d, %d", angle.data, speed.data);
		angle_pub.publish(angle);
		speed_pub.publish(speed);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
