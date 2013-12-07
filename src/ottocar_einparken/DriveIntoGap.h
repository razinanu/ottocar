/*
 * DriveIntoGap.h
 *
 *  Created on: 06.12.2013
 *      Author: jsabsch
 */

#ifndef DRIVEINTOGAP_H_
#define DRIVEINTOGAP_H_


#include "ros/ros.h"

class DriveIntoGap {
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
	twoInts drive(float minimalLaserDistance, float gapSize);

private:

	enum DirectionStatus
	{
		back, forth
	};

	DirectionStatus currentDrivingDirection;
	float minimalLaserDistance;
	float gapSize;

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

	ros::Time currentTime;
	ros::Time lastMark;

	const ros::Duration firstH;
	const ros::Duration spaceback;
	const ros::Duration spacefront;

	//Zeitbasierte erste Hälfte
	bool firstHalfSimulation();

	//zeitbasiert die rückwärtigen Sensoren simulieren
	bool enoughSpaceBackSimulation();

	//zeitbasiert die vorderen Sensoren simulieren
	bool enoughSpaceFrontSimulation();


};

#endif /* DRIVEINTOGAP_H_ */
