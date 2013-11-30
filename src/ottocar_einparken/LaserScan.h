/*
 * LaserScan.h
 *
 *  Created on: Nov 30, 2013
 *      Author: Razi Ghassemi
 */

#ifndef LASERSCAN_H_
#define LASERSCAN_H_


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
using namespace std;
int c ;
float cosinus;
double Pi = 3.14159265;
float distanceHorizontal, baseDistance, baseVDistance, secondHDistance,
			secondVDistance, space, baseHDistance,angle,minVDistance;
const int MINDISTANCE = 0.05;
enum laserState {
	start, calculateBaseRange, calculateMaxPoint, calculateMinPoint, end
};


#endif /* LASERSCAN_H_ */
