/*
 * DriveIntoGap.cpp
 *
 *  Created on: 06.12.2013
 *      Author: jsabsch
 */

#include "DriveIntoGap.h"
#include "ConstPark.h"

DriveIntoGap::DriveIntoGap()
{
	mode = 0;
	lastOdometry = 0;
	SPEED = SPEED_PARKING;
	blinkDone = false;

	lastTimeBlinkerChange = ros::Time::now() - ros::Duration(2);
	blinkerOn = false;
	distanceToDrive = 0;
	gapBegin = ros::Time::now() - ros::Duration(2);

	 ledBlinkerRechts = false;
	 ledBlinkerLinks = false;
	 ledScheinwerferVorneRechts = false;
	 ledScheinwerferVorneLinks = false;
	 ledBremse = false;

	 foundGapSice = 0;
	 ledChanged = false;
}

DriveIntoGap::~DriveIntoGap()
{

}

float DriveIntoGap::drivenM(int odometry)
{
	//gibt die gefahrenen Distanz in m vom letzten Zeitpunkt unabhaengig von der Fahrtrichtung zurueck
	return abs((lastOdometry) - (odometry)) / REVOLUTIONS_PER_M;
}

DriveIntoGap::driveData DriveIntoGap::drive(sensor_msgs::LaserScan laser,
		float gapSize, float distanceBack, float distanceSide, int odometry,
		float voltage, float distanceToGap)
{
	driveData speedAndAngle;
	speedAndAngle.angle.data = STRAIGHTFORWARD;
	speedAndAngle.speed.data = 0;

	switch (mode)
	{
	case 0:
		speedAndAngle = waitForDistance(distanceToGap, odometry, gapSize);
		break;

	case 1:
		speedAndAngle = driveFirstHalf(odometry, distanceSide);
		break;

	case 2:
		speedAndAngle = driveSecondHalf(distanceSide, odometry);
		break;

	case 3:
		speedAndAngle = positioning(odometry, foundGapSice);
		break;

	case 4:
		speedAndAngle = wait1(odometry);
		break;

	case 5:
		speedAndAngle = back1(foundGapSice, odometry);
		break;

	case 6:
		speedAndAngle = wait2(odometry);
		break;

	case 7:
		speedAndAngle = back2(laser, distanceBack, odometry, foundGapSice);
		break;

	case 8:
		speedAndAngle = wait3();
		break;

	case 9:
		speedAndAngle = waitTurn(odometry);
		break;

	case 10:
		speedAndAngle = forwards(laser, foundGapSice, odometry);
		break;

	case 11:
		speedAndAngle = wait4(odometry);
		break;

	case 12:
		speedAndAngle = backLast(distanceBack, odometry);
		break;

	case 13:
		//1. mal an
		ledBlinkerLinks = true;
		ledBlinkerRechts = true;
		ledBremse = true;
		ledChanged = true;
		lastTime = ros::Time::now();
		mode = 14;
		break;

	case 14:
		//1. mal aus
		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
		{
			ledBlinkerLinks = false;
			ledBlinkerRechts = false;
			ledChanged = true;
			lastTime = ros::Time::now();
			mode = 15;
		}
		break;

	case 15:
		//2. mal an
		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
		{
			ledBlinkerLinks = true;
			ledBlinkerRechts = true;
			ledChanged = true;
			lastTime = ros::Time::now();
			mode = 16;
			ROS_INFO("[DIG]: mode auf %d gesetzt", mode);
		}
		break;

	case 16:
		//2. mal aus
		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
		{
			ledBlinkerLinks = false;
			ledBlinkerRechts = false;
			ledChanged = true;
			lastTime = ros::Time::now();
			mode = 17;
			ROS_INFO("[DIG]: mode auf %d gesetzt", mode);
		}
		break;

	case 17:
		//3. mal an
		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
		{
			ledBlinkerLinks = true;
			ledBlinkerRechts = true;
			ledChanged = true;
			lastTime = ros::Time::now();
			mode = 18;
			ROS_INFO("[DIG]: mode auf %d gesetzt", mode);
		}
		break;

	case 18:
		//3. mal aus
		if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
		{
			ledBlinkerLinks = false;
			ledBlinkerRechts = false;
			ledBremse = true;
			ledChanged = true;
			lastTime = ros::Time::now();
			mode = 19;
			ROS_INFO("[DIG]: mode auf %d gesetzt", mode);
		}
		break;

	default:
		ROS_INFO("[DIG]: default mode erreicht");
		speedAndAngle.angle.data = STRAIGHTFORWARD;
		speedAndAngle.speed.data = 0;
		if (!ledBremse)
		{
			ledBremse = true;
			ledChanged = true;
		}
		blinkDone = true;
		break;
	}

	//LEDs schalten
	speedAndAngle.led0.data = 0;
	speedAndAngle.led1.data = 1;
	speedAndAngle.led2.data = 2;
	speedAndAngle.led3.data = 3;
	speedAndAngle.led4.data = 4;
	speedAndAngle.led5.data = 5;
	speedAndAngle.led6.data = 6;
	speedAndAngle.led7.data = 7;
	speedAndAngle.led8.data = 8;

	if (ledBlinkerRechts)
	{
		speedAndAngle.led0.data = 100;
		speedAndAngle.led7.data = 107;
	}
	if (ledBlinkerLinks)
	{
		speedAndAngle.led3.data = 103;
		speedAndAngle.led4.data = 104;
	}
	if (ledScheinwerferVorneLinks)
	{
		speedAndAngle.led2.data = 102;
	}
	if (ledScheinwerferVorneRechts)
	{
		speedAndAngle.led1.data = 101;
	}
	if (ledBremse)
	{
		speedAndAngle.led5.data = 105;
		speedAndAngle.led6.data = 106;
		speedAndAngle.led8.data = 108;
	}

	return speedAndAngle;
}

float DriveIntoGap::calculateSpeed10(float voltage)
{
	float result;
	result = 0.05083733 * voltage - 0.3547296;
	ROS_INFO("[DIG]: voltage: %2.4f | speed: %2.4f", voltage, result);
	return result;
}

DriveIntoGap::driveData DriveIntoGap::wait1(int odometry)
{
	if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
	{
		lastTime = ros::Time::now();
		lastOdometry = odometry;
		mode = 5;
		ledBremse = false;
		ledChanged = true;
	}

	driveData ti;
	ti.angle.data = STRAIGHTFORWARD;
	ti.speed.data = 0;

	return ti;
}

DriveIntoGap::driveData DriveIntoGap::back1(float gapSize, int odometry)
{
	//60 cm
	if (gapSize == (float) 0.6)
	{
		if (drivenM(odometry) > 0.45)
		{
			ROS_INFO("[DIG]: 1. Teil 60cm Rueckwaerts: %2.4f",
					drivenM(odometry));
			lastTime = ros::Time::now();
			lastOdometry = odometry;
			mode = 6;
			ledBremse = true;
			ledChanged = true;
		}

	}

	//70 cm
	else if (gapSize == (float) 0.7)
	{
		if (drivenM(odometry) > 0.43)
		{
			ROS_INFO("[DIG]: 1. Teil 70cm Rueckwaerts: %2.4f",
					drivenM(odometry));
			lastTime = ros::Time::now();
			lastOdometry = odometry;
			mode = 6;
			ledBremse = true;
			ledChanged = true;
		}
	}

	//80 cm
	else if (gapSize == (float) 0.8)
	{
		if (drivenM(odometry) > 0.45)
		{
			ROS_INFO("[DIG]: 1. Teil 80cm Rueckwaerts: %2.4f",
					drivenM(odometry));
			lastTime = ros::Time::now();
			lastOdometry = odometry;
			mode = 6;
			ledBremse = true;
			ledChanged = true;
		}
	}
	else
	{
		ROS_INFO("gapSize: %2.32f", gapSize);
		mode = 25;
	}

	driveData speedAndAngle;
	speedAndAngle.angle.data = RIGHT_MAX;
	speedAndAngle.speed.data = -SPEED;

	return speedAndAngle;
}

DriveIntoGap::driveData DriveIntoGap::wait2(int odometry)
{
	if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
	{
		lastTime = ros::Time::now();
		lastOdometry = odometry;
		mode = 7;
		ledBremse = false;
		ledChanged = true;
	}

	driveData ti;

	ti.speed.data = 0;
	ti.angle.data = STRAIGHTFORWARD;


	if (ledBlinkerRechts && lastTimeBlinkerChange + ros::Duration(0.5) < ros::Time::now())
	{
		ledBlinkerRechts = false;
		ledChanged = true;
		lastTimeBlinkerChange = ros::Time::now();
	}

	return ti;
}

DriveIntoGap::driveData DriveIntoGap::back2(const sensor_msgs::LaserScan laser,
		float distanceBack, int odometry, float gapSize)
{
	//60 cm
	if (gapSize == (float) 0.6)
	{
		float minValue = 1;
		for (int i = 255 - 150; i <= 255 + 150; i++)
		{
			if (laser.ranges[i] < minValue)
			{
				minValue = laser.ranges[i];
			}
		}

		if (minValue > 0.16 && minValue < 0.4 && drivenM(odometry) > 0.22)
		{
			ROS_WARN("[DIG]: in Luecke mit Laserscanner gefahren: %2.4f",
					minValue);
			lastTime = ros::Time::now();
			mode = 8;
			ledBremse = true;
			ledChanged = true;
		}
		//timeOut
		if ((lastTime + ros::Duration(5)) < ros::Time::now())
		{
			ROS_INFO("[DIG]: in die Luecke gefahren: %2.4f", distanceBack);
			ROS_WARN("[DIG]: --> timeOut: %2.4f | odo: %2.4f", minValue,
					drivenM(odometry));
			lastTime = ros::Time::now();
			mode = 8;
			ledBremse = true;
			ledChanged = true;
		}
	}

	//70 cm
	if (gapSize == (float) 0.7)
	{
		float minValue = 1;
		for (int i = 255 - 150; i <= 255 + 150; i++)
		{
			if (laser.ranges[i] < minValue)
			{
				minValue = laser.ranges[i];
			}
		}

		if (minValue > 0.17 && minValue < 0.4 && drivenM(odometry) > 0.25)
		{
			ROS_WARN("[DIG]: in Luecke mit Laserscanner gefahren: %2.4f",
					minValue);
			lastTime = ros::Time::now();
			mode = 8;
			ledBremse = true;
			ledChanged = true;
		}
		//timeOut
		if ((lastTime + ros::Duration(5)) < ros::Time::now())
		{
			ROS_INFO("[DIG]: in die Luecke gefahren: %2.4f", distanceBack);
			ROS_WARN("[DIG]: --> timeOut: %2.4f | odo: %2.4f", minValue,
					drivenM(odometry));
			lastTime = ros::Time::now();
			mode = 8;
			ledBremse = true;
			ledChanged = true;
		}
	}

	//80 cm
	if (gapSize == (float) 0.8)
	{
		float minValue = 1;
		for (int i = 255 - 150; i <= 255 + 150; i++)
		{
			if (laser.ranges[i] < minValue)
			{
				minValue = laser.ranges[i];
			}
		}

		if (minValue > 0.20 && minValue < 0.4 && drivenM(odometry) > 0.25)
		{
			ROS_WARN("[DIG]: in Luecke mit Laserscanner gefahren: %2.4f",
					minValue);
			lastTime = ros::Time::now();
			mode = 8;
			ledBremse = true;
			ledChanged = true;
		}
		//timeout
		if ((lastTime + ros::Duration(5)) < ros::Time::now())
		{
			ROS_INFO("[DIG]: in die Luecke gefahren: %2.4f", distanceBack);
			ROS_WARN("[DIG]: --> timeOut: %2.4f | odo: %2.4f", minValue,
					drivenM(odometry));
			lastTime = ros::Time::now();
			mode = 8;
			ledBremse = true;
			ledChanged = true;
		}
	}

	driveData ti;
	ti.angle.data = LEFT_MAX;
	ti.speed.data = -SPEED;

	return ti;
}

DriveIntoGap::driveData DriveIntoGap::wait3()
{
	if ((lastTime + ros::Duration(0.3)) < ros::Time::now())
	{
		lastTime = ros::Time::now();
		mode = 9;
	}
	driveData ti;
	ti.speed.data = 0;
	ti.angle.data = STRAIGHTFORWARD;

	return ti;
}

DriveIntoGap::driveData DriveIntoGap::waitTurn(int odometry)
{
	if ((lastTime + ros::Duration(0.2)) < ros::Time::now())
	{
		lastTime = ros::Time::now();
		lastOdometry = odometry;
		mode = 10;
		ledBremse = false;
		ledChanged = true;
	}

	driveData ti;
	ti.angle.data = RIGHT_MAX;
	ti.speed.data = 0;

	return ti;
}

DriveIntoGap::driveData DriveIntoGap::forwards(const sensor_msgs::LaserScan laser,
		float gapSize, int odometry)
{
	driveData ti;
	ti.angle.data = STRAIGHTFORWARD;
	ti.speed.data = 0;

	if ((lastTime + ros::Duration(2.0)) < ros::Time::now())	//todo timeout sinnvoll?
	{
		ROS_WARN("[DIG]: nach vorne gefahren: timeOut");
		lastTime = ros::Time::now();
		mode = 11;
		ledBremse = true;
		ledChanged = true;
		return ti;
	}

	//60cm Lücke
	if (gapSize == (float) 0.6)
	{
		for (int i = 255 - 150; i <= 255 + 150; i++)
		{
			if (laser.ranges[i] < 0.13)
			{
				ROS_INFO("[DIG]: nach vorne gefahren: %2.4f | i: %d",
						laser.ranges[i], i);
				lastTime = ros::Time::now();
				mode = 11;
				ledBremse = true;
				ledChanged = true;
				return ti;
			}
		}
		ti.angle.data = 45;
	}

	//70cm Lücke
	if (gapSize == (float) 0.7)
	{
		for (int i = 255 - 150; i <= 255 + 150; i++)
		{
			if (laser.ranges[i] < 0.17)
			{
				ROS_INFO("[DIG]: nach vorne gefahren: %2.4f | i: %d",
						laser.ranges[i], i);
				lastTime = ros::Time::now();
				mode = 11;
				ledBremse = true;
				ledChanged = true;
				return ti;
			}
		}
		ti.angle.data = 25;
	}

	//80 cm
	if (gapSize == (float) 0.8)
	{
		for (int i = 255 - 150; i <= 255 + 150; i++)
		{
			if (laser.ranges[i] < 0.19)
			{
				ROS_INFO("[DIG]: nach vorne gefahren: %2.4f | i: %d",
						laser.ranges[i], i);
				lastTime = ros::Time::now();
				mode = 11;
				ledBremse = true;
				ledChanged = true;
				return ti;
			}
		}
		ti.angle.data = 25;
	}

	ti.speed.data = SPEED;

	return ti;
}

DriveIntoGap::driveData DriveIntoGap::wait4(int odometry)
{
	if ((lastTime + ros::Duration(0.5)) < ros::Time::now())
	{
		lastTime = ros::Time::now();
		lastOdometry = odometry;
		mode = 12;
		ledBremse = false;
		ledChanged = true;
	}

	driveData ti;
	ti.angle.data = STRAIGHTFORWARD;
	ti.speed.data = 0;

	return ti;
}

DriveIntoGap::driveData DriveIntoGap::backLast(float distanceBack, int odometry)
{
	if (drivenM(odometry) > 0.05)
	{
		ROS_INFO("[DIG]: kurzes Stueck zurueckgesetzt: %2.4f", distanceBack);
		ROS_INFO("[DIG]: gefahrene Odometrie: %2.4f", drivenM(odometry));
		lastTime = ros::Time::now();
		mode = 13;
		ledBremse = true;
		ledChanged = true;
	}
	driveData ti;
	ti.angle.data = STRAIGHTFORWARD;
	ti.speed.data = -SPEED;

	return ti;
}

//neu

//warte auf eine Distanz
DriveIntoGap::driveData DriveIntoGap::waitForDistance(float distanceToGap, int odometry, float gapSize)
{
	//wenn eine Lücke akzeptiert wurde, wird auch die Größe der Lücke gespeichert
	if (distanceToGap > 0 && distanceToGap < 1.0)
	{
		foundGapSice = gapSize;
		distanceToDrive = distanceToGap;
		lastOdometry = odometry;
		mode = 1;
	}
	driveData result;
	result.angle.data = STRAIGHTFORWARD;
	result.speed.data = SPEED_DRIVING;
//	if (ledBremse)
//	{
//		ledBremse = false;
//		ledChanged = true;
//	}
	return result;
}

//fahre bis zur Mitte der Lücke
DriveIntoGap::driveData DriveIntoGap::driveFirstHalf(int odometry, float dataIRside)
{
	//bei dem Rückgabewert nur die LEDs betrachten!
	driveData result;

	if (drivenM(odometry) > distanceToDrive - 0.1)
	{
		if (dataIRside > 0.2)
			ROS_INFO(
					"[MTG]: In der Mitte neben der Luecke: %2.2f", dataIRside);
		else
			ROS_WARN(
					"[MTG]: In der Mitte neben der Luecke: %2.2f", dataIRside);

		mode = 2;
	}

	if (!ledBlinkerRechts && lastTimeBlinkerChange + ros::Duration(0.5) < ros::Time::now())
	{
		ledBlinkerRechts = true;
		ledChanged = true;
		lastTimeBlinkerChange = ros::Time::now();
	}

	if (ledBlinkerRechts && lastTimeBlinkerChange + ros::Duration(0.5) < ros::Time::now())
	{
		ledBlinkerRechts = false;
		ledChanged = true;
		lastTimeBlinkerChange = ros::Time::now();
	}

	result.angle.data = STRAIGHTFORWARD;
	result.speed.data = SPEED_DRIVING;
	return result;
}

//auf das Ende der Luecke warten, bis der  IR-Sensor den Karton sieht
DriveIntoGap::driveData DriveIntoGap::driveSecondHalf(float dataIRside, int odometry)
{
	//bei dem Rückgabewert nur die LEDs betrachten!
	driveData result;

	if (dataIRside < 0.2)
	{
		gapBegin = ros::Time::now();
		lastOdometry = odometry;
		mode = 3;
	}

	if (!ledBlinkerRechts && lastTimeBlinkerChange + ros::Duration(0.5) < ros::Time::now())
	{
		ledBlinkerRechts = true;
		ledChanged = true;
		lastTimeBlinkerChange = ros::Time::now();
	}

	if (ledBlinkerRechts && lastTimeBlinkerChange + ros::Duration(0.5) < ros::Time::now())
	{
		ledBlinkerRechts = false;
		ledChanged = true;
		lastTimeBlinkerChange = ros::Time::now();
	}

	result.angle.data = STRAIGHTFORWARD;
	result.speed.data = SPEED_DRIVING;
	return result;
}

//x cm hinter der Luecke anhalten
DriveIntoGap::driveData DriveIntoGap::positioning(int odometry, float gapSize)
{
	driveData result;

	if (drivenM(odometry) > 0.17)
	{
		ROS_INFO("[MTG]: hinter der Luecke angehalten: %2.4f",drivenM(odometry));
		lastTime = ros::Time::now();
		mode = 4;
		result.speed.data = 0;
		result.angle.data = STRAIGHTFORWARD;
		ledBremse = true;
		ledChanged = true;
		return result;
	}

	if (gapSize == (float) 0.8)
	{
		if (drivenM(odometry) > 0.15)
		{
			ROS_INFO("[MTG]: hinter Luecke 80cm angehalten: %2.4f",drivenM(odometry));
			lastTime = ros::Time::now();
			mode = 4;
			result.speed.data = 0;
			result.angle.data = STRAIGHTFORWARD;
			ledBremse = true;
			ledChanged = true;
			return result;
		}
	}

	if (!ledBlinkerRechts && lastTimeBlinkerChange + ros::Duration(0.5) < ros::Time::now())
	{
		ledBlinkerRechts = true;
		ledChanged = true;
		lastTimeBlinkerChange = ros::Time::now();
	}

	if (ledBlinkerRechts && lastTimeBlinkerChange + ros::Duration(0.5) < ros::Time::now())
	{
		ledBlinkerRechts = false;
		ledChanged = true;
		lastTimeBlinkerChange = ros::Time::now();
	}

	result.speed.data = SPEED_PARKING;
	result.angle.data = STRAIGHTFORWARD;

	return result;
}
