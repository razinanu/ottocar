/*
 * ParkingController.h
 *
 *  Created on: Dec 7, 2013
 *      Author: Razi Ghassemi
 */

#ifndef PARKINGCONTROLLER_H_
#define PARKINGCONTROLLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

class ParkingController
{
public:
	ParkingController();
	void LaserScanParkControll(const sensor_msgs::LaserScan laser);
	virtual ~ParkingController();
};

#endif /* PARKINGCONTROLLER_H_ */
