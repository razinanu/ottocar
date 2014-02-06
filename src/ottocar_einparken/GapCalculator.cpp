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
		Pi(3.14159265), gapDistance(-1),gapIs(-1), parkEnable(false)
{

}
bool GapCalculator::testGap(double space)
{
	if (SMALLGAP - 0.05 <= space && space < SMALLGAP + 0.02
			&& minVDistance < 1.00)
	{
		gapIs = 0.60;
		parkEnable = true;
		//ROS_WARN("SMALLGAP FOUND!");
		return true;

	}
	else if (MEDIUMGAP - 0.07 <= space && space < MEDIUMGAP + 0.02
			&& minVDistance < 1.00)
	{
		gapIs =0.70;
		parkEnable = true;
		//ROS_ERROR("MEDIUM FOUND!");
		return true;
	}
	else if (LARGGAP - 0.07 <= space && space <= LARGGAP + 0.02
			&& minVDistance < 1.00)
	{
		gapIs =0.80;
		parkEnable = false;
		//ROS_INFO("LARG FOUND!");
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
		if (currentSearchState == start && HDistance < FIRSTDISTANCE)
		{
			currentSearchState = calculateBaseRange;

		}
		if (currentSearchState == calculateBaseRange)
		{
			if (2 * laser.ranges[i] < laser.ranges[i - 1]
					&& laser.ranges[i] < 1.0 && laser.ranges[i] > 0.19)
			{
				currentSearchState = calculateMaxPoint;
				baseVDistance = laser.ranges[i] * sin(laser.angle_max - angle);
				baseHDistance = laser.ranges[i] * cos(laser.angle_max - angle);
				baseRange = laser.ranges[i];

			}
		}
		if (currentSearchState == calculateMaxPoint
				&& laser.ranges[i] > (laser.ranges[i - 1]))
		{

			currentSearchState = calculateMinPoint;

		}

		if (currentSearchState == calculateMinPoint
				&& laser.ranges[i]
						< (laser.ranges[i - 1] && laser.ranges[i] != BIGRANGE))
		{

			secondHDistance = laser.ranges[i] * cos(laser.angle_max - angle);

			if (abs(secondHDistance - baseHDistance) < MINDISTANCE
					&& abs(baseRange - laser.ranges[i]) > 0.1)
			{
				calculateAverageValue(laser, i);

				space = minVDistance - baseVDistance;

				if (!SEQUENCEPARK)
				{

					simpleParking(space);
				}
				else
				{
					if (testGap(space))
					{
						if (FIRSTPARK)
						{
							gapDistance = minVDistance;
						}
						else if (parkEnable)
						{
							gapDistance = minVDistance;
						}
						else
						{
							currentSearchState = start;
						}
					}
					else
					{
						currentSearchState = start;
					}

				}

			}
		}

		angle -= laser.angle_increment;

	}

}
void GapCalculator::calculateAverageValue(const sensor_msgs::LaserScan laser,
		int i)
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
			secondHDistance2 = abs(
					laser.ranges[i - c]
							* cos(
									laser.angle_increment
											* (laser.ranges.size() - (i - c))));
			secondVDistance2 = abs(
					laser.ranges[i - c]
							* sin(
									laser.angle_increment
											* (laser.ranges.size() - (i - c))));

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
		//ROS_ERROR("GAPDISTANCE: [%f]", gapDistance);

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
