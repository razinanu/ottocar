/*
 * DriveToGap.cpp
 *
 *  Created on: 06.12.2013
 *      Author: licht
 */

#include "DriveToGap.h"

DriveToGap::DriveToGap()
{
}

DriveToGap::~DriveToGap()
{
}

DriveToGap::driveData DriveToGap::driveToGap(float distanceRight, float distanceFront)
{
	driveData result;
	if (distanceFront < 0.3)
	{
		ROS_ERROR("[DTG]: zu weit gefahren 3 %2.4f , rechts: %2.4f", distanceFront, distanceRight);
		result.angle.data = STRAIGHTFORWARD;
		result.speed.data = 0;
		return result;
	}

	if (distanceRight < SET_VALUE_DISTANCE_RIGHT - TOLEARNCE_DISTANCE_RIGHT - TOLEARNCE_DISTANCE_RIGHT - TOLEARNCE_DISTANCE_RIGHT)
	{
		ROS_WARN("[DTG]: nach rechts 2 %2.4f", distanceRight);
		result.angle.data = STRAIGHTFORWARD - 20;
		result.speed.data = -7;
		return result;
	}
	else if (distanceRight < SET_VALUE_DISTANCE_RIGHT - TOLEARNCE_DISTANCE_RIGHT - TOLEARNCE_DISTANCE_RIGHT)
	{
		ROS_WARN("[DTG]: nach rechts %2.4f", distanceRight);
		result.angle.data = STRAIGHTFORWARD - 15;
		result.speed.data = -7;
		return result;
	}
	else if (distanceRight < SET_VALUE_DISTANCE_RIGHT - TOLEARNCE_DISTANCE_RIGHT)
	{
		ROS_WARN("[DTG]: nach rechts %2.4f", distanceRight);
		result.angle.data = STRAIGHTFORWARD - 10;
		result.speed.data = -7;
		return result;
	}
	else if ((distanceRight < DISTANCE_NO_OBSTACLE_RIGHT) && (distanceRight > SET_VALUE_DISTANCE_RIGHT + TOLEARNCE_DISTANCE_RIGHT))
	{
		ROS_WARN("[DTG]: nach links %2.4f", distanceRight);
		result.angle.data = STRAIGHTFORWARD + 10;
		result.speed.data = -7;
		return result;
	}
	else
	{
		ROS_INFO("[DTG]: geradeaus %2.4f", distanceRight);
		result.angle.data = STRAIGHTFORWARD;
		result.speed.data = -7;
		return result;
	}
}
