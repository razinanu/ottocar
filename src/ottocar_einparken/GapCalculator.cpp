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
								Pi(3.14159265), gapDistance(-1),gapIs(-1), parkEnable(false),smallGap(true),largeGap(true),mediumGap(true)
{
	HDistance = -1;
	baseHDistance = -1;
	minVDistance = -1;
	space = -1;
	baseVDistance = -1;
	secondHDistance = -1;
	angle = -1;
	baseRange = -1;
}

bool GapCalculator::testGap(double space)
{
	if (SMALLGAP - 0.05 <= space && space < SMALLGAP + 0.02
			&& smallGap && minVDistance < 1.00)
	{
		gapIs = 0.60;

		parkEnable = true;
		return true;

	}
	else if (MEDIUMGAP - 0.07 <= space && space < MEDIUMGAP + 0.02
			&& mediumGap&&minVDistance < 1.00)
	{
		gapIs = 0.70;
		parkEnable = true;
		return true;
	}
	else if (LARGEGAP - 0.07 <= space && space <= LARGEGAP + 0.02
			&& largeGap&&minVDistance < 1.00)
	{
		gapIs = 0.80;
		parkEnable = false;
		return true;
	}
	return false;
}

void GapCalculator::LaserScanGapCal(const sensor_msgs::LaserScan laser)
{
	minVDistance = 0;
	angle = laser.angle_max;

	for (unsigned int i = laser.ranges.size() - 1;
			i > (laser.ranges.size()) / 2; i--)
	{

		HDistance = laser.ranges[i] * cos(laser.angle_max - angle);

		currentSearchState = startState(currentSearchState);
		currentSearchState = calculateBase(currentSearchState, laser, i);
		currentSearchState = calculateMax(currentSearchState, laser, i);
		currentSearchState = calculateMin(currentSearchState, laser, i);

		angle -= laser.angle_increment;

	}

}

void GapCalculator::calculateAverageValue(const sensor_msgs::LaserScan laser, int i)
{
	double secondHDistance2 = 0;
	double secondVDistance2 = 0;
	double sum = 0;
	double mean = 0;
	int counter = 0;

	for (int c = 0; c < 100; c++)
	{
		if (laser.ranges[i - c] != BIGRANGE)
		{
			secondHDistance2 = abs( laser.ranges[i - c] * cos(laser.angle_increment* (laser.ranges.size() - (i - c))));

			secondVDistance2 = abs(laser.ranges[i - c] * sin( laser.angle_increment * (laser.ranges.size() - (i - c))));

			if (abs(secondHDistance2 - baseHDistance) < MINDISTANCE
					&& secondVDistance2 < 1.0
					&& abs(baseRange - laser.ranges[i - c]) > 0.4)
			{
				sum += secondVDistance2;
				counter++;
			}
		}

	}
	minVDistance = sum / counter;
}

void GapCalculator::simpleParking(double space)
{
	if (BESTGAPLENGTH - 0.05 < space && space < BESTGAPLENGTH + 0.05)
	{
		gapDistance = minVDistance;
	}
	else
	{
		currentSearchState = start;
	}

}

double GapCalculator::getGapDistance()
{
	return gapDistance;
}

GapCalculator::~GapCalculator()
{

}

laserState GapCalculator::startState(laserState currentSearchState)
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
		if (2 * laser.ranges[index] < laser.ranges[index - 1]
		                                           && laser.ranges[index] < 1.0 && laser.ranges[index] > 0.19)
		{
			currentSearchState = calculateMaxPoint;
			baseVDistance = laser.ranges[index] * sin(laser.angle_max - angle);
			baseHDistance = laser.ranges[index] * cos(laser.angle_max - angle);
			baseRange = laser.ranges[index];

			return calculateMaxPoint;
		}
	}

	return currentSearchState;
}
void GapCalculator::foundGap(){
	if(gapIs == 0.6)
	{
		largeGap=false;
		mediumGap=false;

	}
	else if(gapIs == 0.7)
	{

		largeGap=false;
		smallGap=false;
	}
	else if(gapIs == 0.8)
	{
		smallGap=false;
		mediumGap=false;

	}
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

laserState GapCalculator::calculateMin(laserState currentSearchState, const sensor_msgs::LaserScan laser, int index)
{
	if (currentSearchState == calculateMinPoint
			&& laser.ranges[index]
			                < (laser.ranges[index - 1] && laser.ranges[index] != BIGRANGE))
	{

		secondHDistance = laser.ranges[index] * cos(laser.angle_max - angle);

		if (abs(secondHDistance - baseHDistance) < MINDISTANCE
				&& abs(baseRange - laser.ranges[index]) > 0.1)
		{
			calculateAverageValue(laser, index);

			space = minVDistance - baseVDistance;

			if (!SEQUENCEPARK)
			{

				simpleParking(space);
			}
			else
			{
				if (testGap(space))
				{
					ROS_INFO("gapSize: %f", space);

					if (FIRSTPARK)
					{
						foundGap();
						gapDistance = minVDistance;

					}
					else if (parkEnable)
					{
						gapDistance = minVDistance;
					}
					else
					{
						return start;
					}
				}
				else
				{
					return start;
				}

			}

		}
	}

	return currentSearchState;
}
