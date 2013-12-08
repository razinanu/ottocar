/*
 * GapCalculator.cpp
 *
 *  Created on: Dec 6, 2013
 *      Author: Razi Ghassemi
 */

#include "GapCalculator.h"
#include "ConstPark.h"

laserState currentSearchState = start;

GapCalculator::GapCalculator() :
		Pi(3.14159265), c(0)
{

}

bool GapCalculator::LaserScanGapCal(const sensor_msgs::LaserScan laser)
{
	return false;
}
float GapCalculator::minPointFinder(const sensor_msgs::LaserScan laser, int i)
{

	float firstSide = laser.ranges[i];
	float SecondSide = laser.ranges[i + 5];
	float thirdSide;
	return thirdSide;

}

GapCalculator::~GapCalculator()
{

}

