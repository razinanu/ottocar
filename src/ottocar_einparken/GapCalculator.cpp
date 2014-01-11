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

	ROS_WARN("[GAPCAl]: -----------------");
	minVDistance = 0;

	angle = laser.angle_max;
	int counter=0;

	//just as first program, to scan just one time!

//		for (unsigned int i = laser.ranges.size() - 1;
//				i > (laser.ranges.size()) / 2; i--)
//		{
//			HDistance = laser.ranges[i] * cos(laser.angle_max - angle);
//			ROS_INFO(
//					"value: angle=[%f],range[%d]=[%f]: HDitance:%f ", (angle * 180) / Pi, i, laser.ranges[i],HDistance);
//			angle -= laser.angle_increment;
//		}
//
	for (unsigned int i = laser.ranges.size() - 1;
			i > (laser.ranges.size()) / 2; i--)
	{
		counter++;
		HDistance = laser.ranges[i] * cos(laser.angle_max - angle);
		if (currentSearchState == start && HDistance < 0.20)
		{
			currentSearchState = calculateBaseRange;
			ROS_INFO(
					"Start value: angle=[%f],range[%d]=[%f]: ", (angle * 180) / Pi, i, laser.ranges[i]);
		}
		if (currentSearchState == calculateBaseRange)
		{
			if (2 * laser.ranges[i] < laser.ranges[i - 1])
			{
				currentSearchState = calculateMaxPoint;
				baseVDistance = laser.ranges[i] * sin(laser.angle_max - angle);
				//baseVDistance = laser.ranges[i] * sin(counter*laser.angle_increment);
				baseHDistance = laser.ranges[i] * cos(laser.angle_max - angle);
				baseHDistance = laser.ranges[i] * cos(counter*laser.angle_increment);

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
			ROS_INFO(
					"[GAPCAl]: Second Point: angle=[%f],range[%d]=[%f]:  ", (angle * 180) / Pi, i, laser.ranges[i]);
			ROS_INFO("[GAPCAl]: Second Horizontal:%f ", secondHDistance);
			ROS_INFO(
					"[GAPCAl]: DIFFERNCE IS: %f ", abs(secondHDistance-baseHDistance));
			ROS_INFO(" [GAPCAl]:MIND: %f", MINDISTANCE);

			if (abs(secondHDistance - baseHDistance) < MINDISTANCE)
			{

				minVDistance = laser.ranges[i] * sin(laser.angle_max - angle);
				ROS_INFO(
						"[GAPCAl]:MIN value: angle=[%f],range[%d]=[%f]:  ", (angle * 180) / Pi, i, laser.ranges[i]);
				ROS_INFO("[GAPCAl]:MIN Vertical: %f   ", minVDistance);
				space = minVDistance - baseVDistance;
				ROS_INFO("[GAPCAl]: Space is: %f ", space);

				if (0.65 <space&& space < 0.70)
				{
					ROS_INFO("[GAPCAl]: best Gap has been found");
					currentSearchState = end;
					return true;
				}
				else{
					currentSearchState = start;
					return false;
				}
			}
		}

		angle -= laser.angle_increment;

	}

	return false;
}


GapCalculator::~GapCalculator()
{

}
