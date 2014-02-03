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

enum laserState
{
	start,
	calculateBaseRange,
	calculateMaxPoint,
	calculateMinPoint,
	end
};

class GapCalculator
{
private:

	float baseVDistance, secondHDistance,
			space, baseHDistance, angle, minVDistance,
			HDistance, gapDistance;
	double Pi;

	laserState isBaseRange(laserState currentSearchState);
	laserState calculateBase(laserState currentSearchState, const sensor_msgs::LaserScan laser, int index);
	laserState calculateMax(laserState currentSearchState, const sensor_msgs::LaserScan laser, int index);
	bool calculateMin(const sensor_msgs::LaserScan laser, int index);

public:
	GapCalculator();
	virtual ~GapCalculator();
	bool LaserScanGapCal(const sensor_msgs::LaserScan laser);
	float getGapDistance();

};

#endif /* GAPCALCULATOR_H_ */
