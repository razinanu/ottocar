/*
 * Orientation.h
 *
 *  Created on: 07.02.2014
 *      Author: jsabsch
 */

#ifndef ORIENTATION_H_
#define ORIENTATION_H_

#include <ros/ros.h>

class Orientation
{
private:

	float orientX;
	float orientY;
	float orientZ;

	float calculateOrient(float vel, int timeSpan);

public:
	Orientation();
	virtual ~Orientation();

	void updateOrientation(float velX, float velY, float velZ, int timeSpan);
	float orientationX();
	float orientationY();
	float orientationZ();
};

#endif /* ORIENTATION_H_ */
