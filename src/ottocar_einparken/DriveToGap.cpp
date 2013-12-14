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
	result.angle.data = STRAIGHTFORWARD;
	result.speed.data = 0;
	return result;
}
