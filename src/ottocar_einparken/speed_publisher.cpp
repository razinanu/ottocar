#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "einparken_test/Angle.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32.h"
//#include "LaserScan.h"
#include "ConstPark.h"

#include <sstream>
#include <iostream>
float distanceLeft = 2;

enum state
{
	step1_forward, step11, step20, step30, step40, step50, step_end

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
//		std::cout << "[ INFO] [-#-#-#-#-#.-#-#-#-#-]: beendet: "<< new_program_state << " :" << (ros::Time::now() - timeSinceLastChange) << std::endl;
		timeSinceLastChange = ros::Time::now();
		program_State = new_program_state;
	}
}

void driveWithCondition(int speed_value, int angle_value, float time,
		state new_program_state, std_msgs::Int8 &speed, std_msgs::Int16 &angle,
		ros::Time &timeSinceLastChange, state &program_State)
{
	if ((timeSinceLastChange > ros::Time::now() - ros::Duration(time)) /*&& distanceLeft > 0.3*/)
	{
		speed.data = speed_value;
		angle.data = angle_value;
	}
	else
	{
		ROS_INFO("step1 fertig");
		timeSinceLastChange = ros::Time::now();
		program_State = new_program_state;
	}
}

void sensorInput1(std_msgs::Float32 msg)
{
	if (msg.data > 1)
	{
		ROS_WARN("IR_Sensor_1: %4.8f", msg.data);
	}
}

void laserInput(const sensor_msgs::LaserScan laser)
{
//		ROS_INFO(" %2.4f", laser.ranges[laser.ranges.size() - 1]);
		distanceLeft = laser.ranges[laser.ranges.size() - 1];
		if (distanceLeft < 0.3)
		{
			ROS_INFO("[Spe]: %2.4f", distanceLeft);
		}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "speed_publisher");
	ros::NodeHandle n;

	ros::Publisher angle_pub = n.advertise<std_msgs::Int16>("angle_cmd", 1);
	ros::Publisher speed_pub = n.advertise<std_msgs::Int8>("speed_cmd", 1);

	ros::Subscriber sub = n.subscribe("/sensor_IR1", 1, sensorInput1);
	ros::Subscriber hokuyoSubscriber = n.subscribe("/scan", 1, laserInput);

	ros::Rate loop_rate(10);

	ros::Time start = ros::Time::now(); //not used yet
	ros::Time timeSinceLastChange = ros::Time::now();
	std_msgs::Int16 angle;
	std_msgs::Int8 speed;

	state program_State = step_end;

	while (ros::ok())
	{
		switch (program_State)
		{
		//vorwaerts fahren, bis die Parkluecke erreicht wurde
		case (step1_forward):
		{
			driveWithCondition(-8, STRAIGHTFORWARD, 4.0, step_end, speed, angle, timeSinceLastChange,
					program_State);
			break;
		}
		//kleines Stueck weiter fahren
		case (step11):
		{
			drive(-8, STRAIGHTFORWARD, 0.25, step20, speed, angle, timeSinceLastChange,
					program_State);
			break;
		}

			//anhalten
		case (step20):
		{
			drive(0, STRAIGHTFORWARD, 1.0, step30, speed, angle, timeSinceLastChange,
					program_State);
			break;
		}
			//rueckwaerts fahren - rechts lenken
		case (step30):
		{
			drive(8, 430, 1.5, step_end, speed, angle, timeSinceLastChange,
					program_State);
			break;
		}
			//anhalten
		case (step40):
		{
			drive(0, STRAIGHTFORWARD, 1.0, step50, speed, angle, timeSinceLastChange,
					program_State);
			break;
		}
			//rueckwaerts fahren - links lenken
		case (step50):
		{
			drive(8, 310, 1.0, step_end, speed, angle, timeSinceLastChange,
					program_State);
			break;
		}
		default:
		{
			drive(0, STRAIGHTFORWARD, 1.0, step_end, speed, angle, timeSinceLastChange,
					program_State);
			break;
		}
		}

//		ROS_INFO("%d, %d", angle.data, speed.data);
		angle_pub.publish(angle);
		speed_pub.publish(speed);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
