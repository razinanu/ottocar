/*
 * ParkingController.h
 *
 *  Created on: Dec 7, 2013
 *      Author: Simone Bexten
 */

#ifndef PARKINGCONTROLLER_H_
#define PARKINGCONTROLLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "ConstPark.h"

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
	bool rightTurn();
	bool stopTurn();
	void turnDistance(const sensor_msgs::LaserScan laser);

private:
	const float RIGHTTURN;
	const float LEFTTURN;
	const float TOFRONT;
	bool right_turn;
	bool straight_turn;

	float minimalDistance;

	float horizontalDistance;
	float verticalDistance;

	//nan-Werte werden als -1 gespeichert
//	std::vector<float> laserData;
	sensor_msgs::LaserScan laser;

	int findEdge();
};

#endif /* PARKINGCONTROLLER_H_ */
