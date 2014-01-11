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
	start, calculateBaseRange, calculateMaxPoint, calculateMinPoint,followGap, end
};
class GapCalculator
{
private:

	float distanceHorizontal, baseDistance, baseVDistance, secondHDistance,
				secondVDistance, space, baseHDistance,angle,minVDistance,HDistance,gapDistance;
	void  minPointFinder(const sensor_msgs::LaserScan laser);


	double Pi;
	bool writeGapFined;
	int c ;


public:
	GapCalculator();
	virtual ~GapCalculator();
	bool LaserScanGapCal(const sensor_msgs::LaserScan laser);
	float getGapDistance();

};

#endif /* GAPCALCULATOR_H_ */
