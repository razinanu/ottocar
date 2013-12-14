/*
 * DriveToGap.h
 *
 *  Created on: 06.12.2013
 *      Author: licht
 */

#ifndef DRIVETOGAP_H_
#define DRIVETOGAP_H_

#include "ConstPark.h"
#include "ParallelController.h"
#include "Parking.h"

class DriveToGap
{
public:
	struct driveData
	{
		std_msgs::Int8 speed;
		std_msgs::Int16 angle;
	};

	DriveToGap();
	virtual ~DriveToGap();

	driveData driveToGap(float distanceRight, float distanceFront);
private:
};

#endif /* DRIVETOGAP_H_ */
