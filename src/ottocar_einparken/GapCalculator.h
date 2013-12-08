/*
 * GapCalculator.h
 *
 *  Created on: Dec 6, 2013
 *      Author: Razi Ghassemi
 */

#ifndef GAPCALCULATOR_H_
#define GAPCALCULATOR_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
using namespace std;
enum laserState {
	start, calculateBaseRange, calculateMaxPoint, calculateMinPoint, end
};
class GapCalculator
{
private:

	float distanceHorizontal, baseDistance, baseVDistance, secondHDistance,
				secondVDistance, space, baseHDistance,angle,minVDistance,HDistance;
	double Pi;
	int c ;


public:
	GapCalculator();
	virtual ~GapCalculator();
	bool LaserScanGapCal(const sensor_msgs::LaserScan laser);
	float minPointFinder(const sensor_msgs::LaserScan laser,int i);
};

#endif /* GAPCALCULATOR_H_ */
