/*
 * PositionController.h
 *
 *  Created on: Dec 7, 2013
 *      Author: Razi Ghassemi
 */

#ifndef POSITIONCONTROLLER_H_
#define POSITIONCONTROLLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

class PositionController
{
public:
	PositionController();
	void LaserScanPosition(const sensor_msgs::LaserScan laser);
	virtual ~PositionController();
};

#endif /* POSITIONCONTROLLER_H_ */
