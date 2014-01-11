/*
 * DriveToGap.cpp
 *
 *  Created on: 06.12.2013
 *      Author: licht
 */

#include "MoveToGap.h"

MoveToGap::MoveToGap()
{
	cartonSeen = false;
	distanceGot = false;
	gapBegin = ros::Time::now() - ros::Duration(2);
}

MoveToGap::~MoveToGap()
{
}

MoveToGap::driveData MoveToGap::moveToGap(float dataIR, float distanceToGap)
{
	driveData result;

	result.angle.data = -21;
	result.speed.data = -8;

	if (distanceToGap > 0)
	{
		if (distanceToGap > 0 && !distanceGot)
		{
			distanceFound = ros::Time::now();
			distanceGot = true;
			timeToDrive = (distanceToGap / 0.4);
		}

		if ((distanceFound + ros::Duration(timeToDrive)) > ros::Time::now())
		{

		}
		else
		{
			//Zeit nehmen, wenn Karton gesehen
			if (dataIR < 12 && !cartonSeen)
			{
				gapBegin = ros::Time::now();
				cartonSeen = true;
				ROS_INFO("[MTG]: Karton gesehen");
			}
			else if (!cartonSeen)
			{
				gapBegin = ros::Time::now();
			}

			//verzögert anhalten
			if (gapBegin < (ros::Time::now() - ros::Duration(0.5)))
			{
				result.speed.data = 0;
			}
		}
	}

	return result;
}
