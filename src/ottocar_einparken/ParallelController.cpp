/*
 * ParallelController.cpp
 *
 *  Created on: Dec 6, 2013
 *      Author: licht
 */

#include "ParallelController.h"

ParallelController::ParallelController()
{
	obstacleInFrontOfTheCar = true;
}

ParallelController::~ParallelController()
{

}

void ParallelController::laserScanParallel(const sensor_msgs::LaserScan laser)
{
	check1(laser);

	//pruefen, ob vor dem Auto ein Hindernis steht (nicht elegant)
	if (laser.ranges[laser.ranges.size() - (255 + 1)] < 0.3)
	{
		check2();
		obstacleInFrontOfTheCar = true;
	}
	else
	{
		obstacleInFrontOfTheCar = false;
	}
}

bool ParallelController::driveEnable()
{
	return !obstacleInFrontOfTheCar;
}

void ParallelController::check1(const sensor_msgs::LaserScan laser)
{
#if LASER_DATA == true
	for (int i = 0; i <= (SEARCH_SPACE); i++)
	{
		ROS_INFO(
				"i: %d: %2.4f | %2.4f", i, laser.ranges[511 - i], ((TARGET_DISTANCE / cos(i * laser.angle_increment)) * TOLERANCE));
	}
#endif
}

void ParallelController::check2()
{
#if INFO == true
		if (!obstacleInFrontOfTheCar)
			ROS_WARN("[PLC]: Hindernis erkannt");
#endif
}
