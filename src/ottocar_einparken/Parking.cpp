/*
 * Parking.cpp
 *
 * Created on: Dec 6, 2013
 * Author: Razi Ghassemi
 */

#include "Parking.h"

Parking::Parking() :
		GapCalculator_(true), ParallelController_(true), ParkingController_(
				false)
{
	intoGapAngle = 0;
	intoGapSpeed = 0;
	distanceBack = -1;
	distanceSide = -1;

	motorRevolutions = 0;

	bufferBack = new RingBuffer();
	bufferSide = new RingBuffer();

	lastLaserscanTime = ros::Time::now();
	button = false;
}

Parking::~Parking()
{
	bufferBack->~RingBuffer();
	bufferSide->~RingBuffer();
}

void Parking::scanValues(sensor_msgs::LaserScan laser)
{
//todo each class create a COPY of Laser data, if it changes the data!

	for (unsigned int i = 0; i < laser.ranges.size(); i++)
	{
		if (laser.ranges[i] == INFINITY || isnan(laser.ranges[i]))
		{
			laser.ranges[i] = BIGRANGE;
		}
	}

//as long as the best Gap was found
	if (GapCalculator_)
	{
		gapcal.LaserScanGapCal(laser);
		gapDistance = gapcal.gapIs;
	}

	if (ParallelController_)
	{
		parallel.laserScanParallel(laser);
	}

	lastLaserscanTime = ros::Time::now();
	g_laser = laser;

}

float Parking::linearizeBack(float value)
{
	float result = 0.1194 / (value + 0.028);

	if (result > 0 && result <= 0.4)
	{
		return result;
	}
	else
	{
		return 0.4;
	}
}

float Parking::linearizeSide(float value)
{
	float result = 0.1128 / (value - 0.124);

	if (result > 0 && result <= 0.4)
	{
		return result;
	}
	else
	{
		return 0.4;
	}

}

void Parking::ir1Values(std_msgs::Float32 sensor)
{
// this->distanceBack = linearizeBack(sensor.data);
	bufferSide->insert(linearizeBack(sensor.data));
}

void Parking::ir2Values(const std_msgs::Float32 sensor)
{
// this->distanceSide = linearizeSide(sensor.data);
	bufferBack->insert(linearizeSide(sensor.data));
}

void Parking::voltageValues(std_msgs::Float32 msg)
{
	voltage = msg.data;
//todo Klasse Ringpuffer benutzen ;)
}

void Parking::motorValues(const std_msgs::Int32 sensor)
{
	motorRevolutions = sensor.data;
}

void Parking::buttonPressed(const std_msgs::Bool msg)
{
	button = !button;
}

void Parking::initButton()
{
	buttonOnCar_Subscriber = parkingNode.subscribe("/GPIO_button1", 1, &Parking::buttonPressed, this);
}

void Parking::init()
{
	angle_pub = parkingNode.advertise<std_msgs::Int8>("angle_cmd", 1);
	speed_pub = parkingNode.advertise<std_msgs::Int8>("speed_cmd", 1);
	led_pub = parkingNode.advertise<std_msgs::UInt8>("led_set", 30);

	hokuyoSubscriber = parkingNode.subscribe("/scan", 1, &Parking::scanValues,
			this);
	sensor_ir1_Subscriber = parkingNode.subscribe("/sensor_IR1", 1,
			&Parking::ir1Values, this);
	sensor_ir2_Subscriber = parkingNode.subscribe("/sensor_IR2", 1,
			&Parking::ir2Values, this);
	sensor_voltage = parkingNode.subscribe("/sensor_voltage", 1,
			&Parking::voltageValues, this);
	sensor_motor_revolutions_Subscriber = parkingNode.subscribe(
			"/sensor_motor_revolutions", 1, &Parking::motorValues, this);

	ros::Duration(1).sleep();
}

void Parking::finishedParkLed()
{
	// anschalten
	if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
	{
		msg_led.data = 0; // vorne rechts
		led_pub.publish(msg_led);
		msg_led.data = 7; // hinten rechts
		led_pub.publish(msg_led);
		msg_led.data = 3; // vorne links
		led_pub.publish(msg_led);
		msg_led.data = 4; // hinten links
		led_pub.publish(msg_led);
	}

	if ((lastTime + ros::Duration(1.0)) < ros::Time::now())
	{
		msg_led.data = 100; // vorne rechts
		led_pub.publish(msg_led);
		msg_led.data = 107; // hinten rechts
		led_pub.publish(msg_led);
		msg_led.data = 103; // vorne links
		led_pub.publish(msg_led);
		msg_led.data = 104; // hinten links
		led_pub.publish(msg_led);

		lastTime = ros::Time::now();
		count++;
	}
}

// alle Lichter ausschalten
void Parking::allLightsOff()
{
	for (int i = 0; i < 10; ++i)
	{
		msg_led.data = i;
		led_pub.publish(msg_led);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "parking");

	Parking park;
	park.initButton();

	//licht aus
	park.allLightsOff();

	//erst loslegen, wenn das vom Button gesendet wurde
	while(!park.button)
	{
		ros::spinOnce();
	}

	try
	{
		park.init();
	} catch (std::exception& error)
	{
		ROS_ERROR("Error: %s\n", error.what());
		return -1;
	} catch (...)
	{
		ROS_ERROR("Unknown Error\n\r");
		return -1;
	}


	DriveIntoGap::driveData data;

	//Bremslicht an
	data.led1.data = 105;
	data.led2.data = 106;
	data.led3.data = 108;
	park.led_pub.publish(data.led1);
	park.led_pub.publish(data.led2);
	park.led_pub.publish(data.led3);

	//Scheinwerfer an
	data.led2.data = 101;
	data.led3.data = 102;
	park.led_pub.publish(data.led1);
	park.led_pub.publish(data.led2);

	data.angle.data = 0;
	data.speed.data = 0;
	park.angle_pub.publish(data.angle);
	park.speed_pub.publish(data.speed);
	ros::Duration(0.4).sleep();

	data.angle.data = -90;
	park.angle_pub.publish(data.angle);
	ros::Duration(0.4).sleep();

	data.angle.data = 0;
	data.speed.data = 0;
	park.angle_pub.publish(data.angle);
	park.speed_pub.publish(data.speed);
	ros::Duration(0.4).sleep();

	ROS_INFO("[PAR]: Parking gestartet");

	ros::Rate loop_rate(LOOP_RATE);

	//licht aus
	park.allLightsOff();

	//Scheinwerfer an
	data.led2.data = 101;
	data.led3.data = 102;
	park.led_pub.publish(data.led1);
	park.led_pub.publish(data.led2);

	park.count = 0;

	ros::spinOnce();

	while (ros::ok)
	{
		//beenden, wenn der button wieder gedrückt wurde
		if(!park.button)
		{
			return 0;
		}

//		//move to the gap
//		if (!park.ParkingController_)
//		{
//			if (park.parallel.driveEnable())
//			{
//				data = driver.moveToGap(park.g_laser,
//						park.bufferSide->getMedian(),
//						park.bufferBack->getMedian(),
//						park.gapcal.getGapDistance(), park.voltage,
//						park.motorRevolutions, park.gapcal.gapIs);
//
//				if (data.speed.data == 0)
//				{
//					park.ParkingController_ = true;
//				}
//			}
//			else
//			{
//				data.speed.data = 0;
//			}
//			park.angle_pub.publish(data.angle);
//			park.speed_pub.publish(data.speed);
//			park.led_pub.publish(data.led1);
//			park.led_pub.publish(data.led2);
//			park.led_pub.publish(data.led3);
//		}
//		//drive into the gap
//		else if (park.ParkingController_)
//		{

		if (park.driveIntoGap.mode == 2)
		{
			park.ParkingController_ = true;
		}

		DriveIntoGap::driveData data = park.driveIntoGap.drive(park.g_laser,
				park.gapcal.gapIs, park.bufferBack->getMedian(),
				park.bufferSide->getMedian(), park.motorRevolutions,
				park.voltage, park.gapcal.getGapDistance());

		park.angle_pub.publish(data.angle);
		park.speed_pub.publish(data.speed);

		if (park.driveIntoGap.ledChanged)
		{
			park.led_pub.publish(data.led0);
			park.led_pub.publish(data.led1);
			park.led_pub.publish(data.led2);
			park.led_pub.publish(data.led3);
			park.led_pub.publish(data.led4);
			park.led_pub.publish(data.led5);
			park.led_pub.publish(data.led6);
			park.led_pub.publish(data.led7);
			park.led_pub.publish(data.led8);
			park.driveIntoGap.ledChanged = false;
		}

		// Programm fertig; LEDs ausschalten
		if (park.driveIntoGap.blinkDone)
		{
			data.led0.data = 0;
			data.led1.data = 1;
			data.led2.data = 2;
			data.led3.data = 3;
			data.led4.data = 4;
			data.led5.data = 5;
			data.led6.data = 6;
			data.led7.data = 7;
			data.led8.data = 8;
			park.led_pub.publish(data.led0);
			park.led_pub.publish(data.led1);
			park.led_pub.publish(data.led2);
			park.led_pub.publish(data.led3);
			park.led_pub.publish(data.led4);
			park.led_pub.publish(data.led5);
			park.led_pub.publish(data.led6);
			park.led_pub.publish(data.led7);
			park.led_pub.publish(data.led8);

			ROS_WARN("[PAR]: Parking beendet :)");
			return 0;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	ROS_WARN("[PAR]: Parking beendet");
	return 0;
}
