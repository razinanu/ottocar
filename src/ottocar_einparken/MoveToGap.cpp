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
	begin = ros::Time::now()	- ros::Duration(2);
}

MoveToGap::~MoveToGap()
{
}

MoveToGap::driveData MoveToGap::moveToGap(float dataIR)
{
	driveData result;

	result.angle.data = -21;
	result.speed.data = -8;

	//Zeit nehmen, wenn Karton gesehen
	if (dataIR < 12 && !cartonSeen)
	{
		begin = ros::Time::now();
		cartonSeen = true;
		ROS_INFO("[MTG]: Karton gesehen");
	}
	else if (!cartonSeen)
	{
		begin = ros::Time::now();
	}

	//verzögert anhalten
	if (begin	< (ros::Time::now()	- ros::Duration(0.5)))
	{
		result.speed.data = 0;
	}

	return result;
}
