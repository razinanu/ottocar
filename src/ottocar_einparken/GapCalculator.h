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
enum GAPSIZE{
	def,sGAP,mGAP,lGAP,

};

/**\brief  This class is to detect the best gap
 *
 * To detect the Gap, at first searching the first corner of Obstacle, which has
 * distance smaller than FIRSTDISTANCE (see ConstPark.h) to laser scanner. Save his horizontal and vertical distance to
 * Laser scanner as base point. Next searching the corner of second obstacle to calculate the gap
 * length. By each found point compare  difference between his horizontal distance and base horizontal
 * distance, if it lies in the interval MINDISTANCE, accept it and calculate the difference of his vertical and base vestcal
 * distance to determine length of gap.
 **/
class GapCalculator
{
private:

	/**\brief The vertical distance of first obstacle to current location of car **/
	double baseVDistance,
	/**\brief The horizontal distance of first obstacle to current location of car **/
	baseHDistance,
	/**\brief The horizontal distance of second obstacle to current location of car **/
	secondHDistance,
	/**\brief The size of gap **/
	space,
	/**\brief Starting range of laser scanner that recorded Obstacle**/
	baseRange,
	/**\brief Scanning angle in radian **/
	angle,
	minVDistance,
	HDistance,
	/**\brief The distance of car to gap **/
	gapDistance;
	double Pi;

	bool parkEnable,smallGap,mediumGap,largeGap;
	/**\brief If parking is based on the length of Gap
	 * \param double The length of gap
	 * **/
	void simpleParking(double space);
	/**\brief Calculate the mean value of distance for the corner of second obstacle
	 * \param sensor_msgs::LaserScan The laser scanner data
	 * \param int Starting range of laser scanner that recorded Obstacle
	 *  **/
	void calculateAverageValue(const sensor_msgs::LaserScan laser, int i);
	/**\brief To decide which of three possible gaps has been detected
		 * \param double The length of gap
		 * **/
	bool testGap(double space);
	/**\brief To publish which gap has been found**/
	void foundGap();

	/**\brief Start the search
	 *\param laserState The current search state
 	**/
	laserState startState(laserState currentSearchState);
	/**\brief To calculate the end corner of the first near obstacle
	 * \param laserState the current search state
 	 * \param sensor_msgs::LaserScan the laser scanner data
	 * \param int starting range of laser scanner that recorded Obstacle
	 *  **/
	laserState calculateBase(laserState currentSearchState, const sensor_msgs::LaserScan laser, int i);
	/**\brief To calculate the second obstacle
	 * \param laserState The current search state
 	 * \param sensor_msgs::LaserScan The laser scanner data
	 * \param int Starting range of laser scanner that recorded Obstacle
	**/
	laserState calculateMax(laserState currentSearchState, const sensor_msgs::LaserScan laser, int i);
	/**\brief To calculate the front corner of the second obstacle
 	 * \param laserState The current search state
 	 * \param sensor_msgs::LaserScan The laser scanner data
	 * \param int Starting range of laser scanner that recorded Obstacle
	 * **/
	laserState calculateMin(laserState currentSearchState, const sensor_msgs::LaserScan laser, int i);

public:
	GapCalculator();
	virtual ~GapCalculator();
	/**\brief This function receives regularly the laser scan data and searches for important points
	 * in the region of interest (the right range of laser scan from 0 to 90 degree) **/
	void LaserScanGapCal(const sensor_msgs::LaserScan laser);
	/**\brief return distance of car to the best gap which has been found **/
	double getGapDistance();
	/**\brief The size of gap **/
	double gapIs;

};

#endif /* GAPCALCULATOR_H_ */
