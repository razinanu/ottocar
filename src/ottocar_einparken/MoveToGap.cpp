/*
 * DriveToGap.cpp
 *
 *  Created on: 06.12.2013
 *      Author: licht
 */

#include "MoveToGap.h"

MoveToGap::MoveToGap()
{
}

MoveToGap::~MoveToGap()
{
}

MoveToGap::driveData MoveToGap::moveToGap(ParallelController::orientationData data)
{
	driveData result;
	result.speed.data = 0;

	int angleValue = (TARGET_DISTANCE - data.distance) * 1000 * (-1);
	if (angleValue < 50)
	{
//		result.angle.data = angleValue * 2;
//		result.speed.data = 6;

		result.angle.data = 0;
		result.speed.data = 0;

//		ROS_INFO("[MTG]: Lenkung: %d", angleValue);
	}
	else
	{
		result.angle.data = 0;
		result.speed.data = 0;
//		ROS_WARN("[MTG]: Lenkung: %d", angleValue);
	}



	return result;
}
