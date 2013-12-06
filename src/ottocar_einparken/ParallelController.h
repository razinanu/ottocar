/*
 * ParallelController.h
 *
 *  Created on: Dec 6, 2013
 *      Author: Razi Ghassemi
 */

#ifndef PARALLELCONTROLLER_H_
#define PARALLELCONTROLLER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

class ParallelController
{
public:
	ParallelController();
	void LaserScanParallel(const sensor_msgs::LaserScan laser);
	virtual ~ParallelController();

};

#endif /* PARALLELCONTROLLER_H_ */
