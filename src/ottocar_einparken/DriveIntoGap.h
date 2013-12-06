/*
 * DriveIntoGap.h
 *
 *  Created on: 06.12.2013
 *      Author: jsabsch
 */

#ifndef DRIVEINTOGAP_H_
#define DRIVEINTOGAP_H_

class DriveIntoGap {
public:

	struct twoInts
	{
		int x;
		int y;
	};

	DriveIntoGap();
	virtual ~DriveIntoGap();

	/**
	 * [0]: angle
	 * [1]: speed
	 */
	twoInts drive();

private:
	/**
	 * Is the Robot able to drive backwards? Or is the wall already too close?
	 */
	bool enoughSpaceInTheBack();

	/**
	 * drive backwards in a fine S-Movement
	 */
	twoInts backward();

	/**
	 * drive forwards in a nearly straight line while correcting your direction
	 */
	twoInts forward();

	/**
	 * Befindet sich der Roboter in der vorderen Hälfte der Lücke?
	 */
	bool firstHalf();

	/**
	 * Roboter ist ungefähr parallel zur Straße ausgerichtet
	 */
	bool isStraight();

	//###################################################
	//temporär, bis sinnvolle Methoden von Simone kommen:
	//###################################################


};

#endif /* DRIVEINTOGAP_H_ */
