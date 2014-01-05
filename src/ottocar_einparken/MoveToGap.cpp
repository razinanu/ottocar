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

MoveToGap::driveData MoveToGap::moveToGap(float distanceRight, float distanceFront)
{
	driveData result;
	result.angle.data = STRAIGHTFORWARD;
	result.speed.data = 0;
	return result;
}
