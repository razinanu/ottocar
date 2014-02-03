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
		Pi(3.14159265), gapDistance(-1)
{
	HDistance = -1;
	baseHDistance = -1;
	minVDistance = -1;
	space = -1;
	baseVDistance = -1;
	secondHDistance = -1;
	angle = -1;
}

laserState GapCalculator::isBaseRange(laserState currentSearchState)
{
	if (currentSearchState == start && HDistance < FIRSTDISTANCE)
	{
		return calculateBaseRange;
	}

	return currentSearchState;
}

laserState GapCalculator::calculateBase(laserState currentSearchState, const sensor_msgs::LaserScan laser, int index)
{
	if (currentSearchState == calculateBaseRange)
	{
		if (2 * laser.ranges[index] < laser.ranges[index - 1])
		{

			baseVDistance = laser.ranges[index] * sin(laser.angle_max - angle);
			baseHDistance = laser.ranges[index] * cos(laser.angle_max - angle);

			return calculateMaxPoint;
		}
	}

	return currentSearchState;
}

laserState GapCalculator::calculateMax(laserState currentSearchState, const sensor_msgs::LaserScan laser, int index)
{
	if (currentSearchState == calculateMaxPoint
			&& laser.ranges[index] > (laser.ranges[index - 1]))
	{
		return calculateMinPoint;
	}

	return currentSearchState;
}

bool GapCalculator::calculateMin(const sensor_msgs::LaserScan laser, int index)
{
	if (currentSearchState == calculateMinPoint
			&& laser.ranges[index] < (laser.ranges[index - 1]))
	{
		secondHDistance = laser.ranges[index] * cos(laser.angle_max - angle);

		if (abs(secondHDistance - baseHDistance) < MINDISTANCE)
		{
			minVDistance = laser.ranges[index] * sin(laser.angle_max - angle);
			space = minVDistance - baseVDistance;

			if (BESTGAPLENGTH - 0.05 < space && space < BESTGAPLENGTH + 0.05)
			{
				gapDistance = minVDistance;
			}
			else
			{
				currentSearchState = start;
				return false;
			}
		}
	}

	return true;
}

bool GapCalculator::LaserScanGapCal(const sensor_msgs::LaserScan laser)
{
	minVDistance = 0;
	angle = laser.angle_max;

	for (unsigned int i = laser.ranges.size() - 1; i > (laser.ranges.size()) / 2; i--)
	{
		HDistance = laser.ranges[i] * cos(laser.angle_max - angle);

		currentSearchState = isBaseRange(currentSearchState);
		currentSearchState = calculateBase(currentSearchState, laser, i);
		currentSearchState = calculateMax(currentSearchState, laser, i);

		if(!calculateMin(laser, i))
			return false;

		angle -= laser.angle_increment;
	}

	return false;
}

float GapCalculator::getGapDistance()
{
	return gapDistance;
}

GapCalculator::~GapCalculator()
{

}
