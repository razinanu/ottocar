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
#include "std_msgs/Float32.h"

class ParkingController
{
public:
	ParkingController();
	void LaserScanParkControll(const sensor_msgs::LaserScan laser);
	virtual ~ParkingController();

	//distance to the front wall
	float getFrontDistance();
	float getMinimalDistance();

	//is the car at the front half of the gap?
	bool rightTurn();
	bool stopTurn();

private:
	const float RIGHTTURN;
	const float LEFTTURN;
	const float TOFRONT;
	bool right_turn;
	bool left_turn;
	bool straight_turn;

	float minimalDistance;

	float horizontalDistanceToObstacle;
	float verticalDistanceToObstacle;

	//nan-Werte werden als -1 gespeichert
	sensor_msgs::LaserScan laser;

	int findMinEdge(int);
	int findMaxForHorizontal();
	int findMaxForVertical();

	float getDistanceToStreet();
	float computeTriangulationForDistance(float rayAtMinEdge,
			float rayAccordingToEdge, float angleGamma);
	void turnOver();
};

#endif /* PARKINGCONTROLLER_H_ */
