/*
 * DriveToGap.h
 *
 *  Created on: 06.12.2013
 *      Author: licht
 */

#ifndef MOVETOGAP_H_
#define MOVETOGAP_H_

#include "ConstPark.h"
#include "ParallelController.h"
#include "Parking.h"

class MoveToGap
{
public:
	struct driveData
	{
		std_msgs::Int8 speed;
		std_msgs::Int8 angle;
	};

	MoveToGap();
	virtual ~MoveToGap();

	driveData moveToGap(float dataIR);
private:
	bool cartonSeen;
	ros::Time begin;
};

#endif /* MOVETOGAP_H_ */
