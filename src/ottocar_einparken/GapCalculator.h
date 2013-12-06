/*
 * GapCalculator.h
 *
 *  Created on: Dec 6, 2013
 *      Author: razi
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
				secondVDistance, space, baseHDistance,angle,minVDistance;
	double Pi;
	int c ;


public:
	GapCalculator();
	virtual ~GapCalculator();
	void LaserScanGapCal(const sensor_msgs::LaserScan laser);
};

#endif /* GAPCALCULATOR_H_ */
