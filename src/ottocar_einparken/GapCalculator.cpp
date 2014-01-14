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

}

bool GapCalculator::LaserScanGapCal(const sensor_msgs::LaserScan laser)
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
			if (2 * laser.ranges[i] < laser.ranges[i - 1])
			{
				currentSearchState = calculateMaxPoint;
				baseVDistance = laser.ranges[i] * sin(laser.angle_max - angle);
				baseHDistance = laser.ranges[i] * cos(laser.angle_max - angle);
			    ROS_INFO(

			           "BaseRange value: angle=[%f],range[%d]=[%f]:  ", (angle * 180) / Pi, i, laser.ranges[i]);

			     ROS_INFO(

			           "Next point after BaseRange value: angle=[%f],range[%d]=[%f]:  ", (angle * 180) / Pi, i-1, laser.ranges[i-1]);

			      ROS_INFO(

		           "BaseRange Horizontal:%f Vertical:%f   ", baseHDistance, baseVDistance);

			}
		}
		if (currentSearchState == calculateMaxPoint
				&& laser.ranges[i] > (laser.ranges[i - 1]))
		{
			currentSearchState = calculateMinPoint;
		}

		if (currentSearchState == calculateMinPoint
				&& laser.ranges[i] < (laser.ranges[i - 1]))
		{

			secondHDistance = laser.ranges[i] * cos(laser.angle_max - angle);

			if (abs(secondHDistance - baseHDistance) < MINDISTANCE)
			{

				minVDistance = laser.ranges[i] * sin(laser.angle_max - angle);
				space = minVDistance - baseVDistance;
				ROS_INFO(

						"[GAPCAl]:MIN value: angle=[%f],range[%d]=[%f]:  ", (angle * 180) / Pi, i, laser.ranges[i]);

				ROS_INFO("[GAPCAl]:MIN Vertical: %f   ", minVDistance);

				ROS_INFO("[GAPCAl]: Space is: %f ", space);

				if (BESTGAPLENGTH - 0.05 < space && space < BESTGAPLENGTH + 0.05)
				{
					gapDistance = minVDistance;
					ROS_ERROR("GAPDISTANCE: [%f]", gapDistance);

				}
				else
				{
					currentSearchState = start;
					return false;
				}
			}
		}

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
