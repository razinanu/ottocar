/*
 * GapCalculator.h
 *
 *  Created on: Dec 6, 2013
 *      Author: Razi Ghassemi
 */
#ifndef GAPCALCULATOR_H_
#define GAPCALCULATOR_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
using namespace std;
enum laserState
{
	start, calculateBaseRange, calculateMaxPoint, calculateMinPoint, end
};
//enum GAPIS
//{
//	smallGap, mediumGap, largGap,
//
//};
/**\brief
 * To detect the Gap, at first searching the first corner of Obstacle, which has
 * distance smaller than FIRSTDISTANCE to laser scanner. Save his horizontal and vertical distance to
 * Laser scanner as base point. Next searching the corner of second obstacle to calculate the gap
 * length. By each found point compare  difference between his horizontal distance and base horizontal
 * distance, if it lies in the interval MINDISTANCE, accept it and calculate the difference of his vertical and base vestcal
 * distance to determine length of gap.
 **/

class GapCalculator
{
private:

	double distanceHorizontal, baseDistance, baseVDistance, secondHDistance,
			secondVDistance, space, baseHDistance, angle, minVDistance,
			HDistance, gapDistance, baseRange;
	double Pi;

	bool parkEnable;
	/**\brief if for parking just consider the length of Gap
	 * \param double space the length of gap
	 * **/
	void simpleParking(double space);
	/**\brief calculate the mean value of distance for the corner of second obstacle
	 * \param double space the length of gap
	 * **/
	void calculateAverageValue(const sensor_msgs::LaserScan laser, int i);

	bool testGap(double space);

public:
	GapCalculator();
	virtual ~GapCalculator();
	void LaserScanGapCal(const sensor_msgs::LaserScan laser);
	double getGapDistance();
	/**\brief size of gap **/
	double gapIs;

};

#endif /* GAPCALCULATOR_H_ */
