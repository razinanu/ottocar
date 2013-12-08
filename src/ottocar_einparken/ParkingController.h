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

	//distance to the front wall
	float getFrontDistance();

	//distance to the back wall
	float getBackDistance();

	//search for the minimal distance of your scanranges and return it
	float getMinimalDistance();

	//is the car at the front half of the gap?
	bool firstHalf();

private:

	float minimalDistance;

	//nan-Werte werden als -1 gespeichert
	std::vector<float> laserData;
};

#endif /* PARKINGCONTROLLER_H_ */
