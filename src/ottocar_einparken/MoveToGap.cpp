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

	int angleValue = (TARGET_DISTANCE - data.distance) * 500 * (-1) * 1.2;
	if (angleValue < 120)
	{
//		result.angle.data = angleValue;
		result.speed.data = -8;

		result.angle.data = -21;
//		result.speed.data = 0;

		ROS_INFO("[MTG]: Lenkung: %d | %2.4f", angleValue, data.distance);
	}
	else
	{
		result.angle.data = -21;
		result.speed.data = -8;
		ROS_WARN("[MTG]: Lenkung: %d | %2.4f", angleValue, data.distance);
	}



	return result;
}
