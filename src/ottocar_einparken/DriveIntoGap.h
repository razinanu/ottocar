/*
 * DriveIntoGap.h
 *
 *  Created on: 06.12.2013
 *      Author: jsabsch
 */

#ifndef DRIVEINTOGAP_H_
#define DRIVEINTOGAP_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "ParkingController.h"
#include "std_msgs/Float32.h"

class DriveIntoGap
{
public:

	struct twoInts
	{
		int x;
		int y;
	};

	DriveIntoGap();
	virtual ~DriveIntoGap();

	/**
	 * [1]: angle
	 * [0]: speed
	 */
	twoInts drive(sensor_msgs::LaserScan laser, float gapSize, float distanceBack, float distanceSide);

private:

	enum DirectionStatus
	{
		back, forth
	};

	float minimalLaserDistance;
	DirectionStatus currentDrivingDirection;
	float gapSize;
	ParkingController parkingController;
	/**
	 * Is the Robot able to drive backwards? Or is the wall already too close?
	 */
	bool enoughSpaceInTheBack();

	/**
	 * Is the Robot able to drive forwards? Or ist the wall already too close?
	 */
	bool enoughSpaceOnTheFront();

	/**
	 * drive backwards in a fine S-Movement
	 */
	twoInts backward();

	/**
	 * drive forwards in a nearly straight line while correcting your direction
	 */
	twoInts forward();

	/**
	 * Befindet sich der Roboter in der vorderen Hälfte der Lücke?
	 */
	bool firstHalf();

	/**
	 * Roboter ist ungefähr parallel zur Straße ausgerichtet
	 */
	bool isStraight();

	//###################################################
	//temporär, bis sinnvolle Methoden von Simone kommen:
	//###################################################

};

#endif /* DRIVEINTOGAP_H_ */
