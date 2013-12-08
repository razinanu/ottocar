/*
 * ConstPark.h
 *
 *  Created on: Dec 6, 2013
 *      Author: Razi Ghassemi
 */

#ifndef PARK_H_
#define PARK_H_

/*
 * Loop rates
 */
const int LOOP_RATE = 50;

//ParkingConst
const float BIGRANGE=99.0;
const float PARALLELDISTANCE=0.4;

//GapCalculatorConst
const float MINDISTANCE = 0.05;
const float FIRSTDISTANCE = 0.3;

//Lenkwerte
const int STRAIGHTFORWARD = 383;
const int LEFT_MAX = 310;
const int RIGHT_MAX = 430;

//DriveIntoGap
const float SECUREDISTANCE_BACK = 0.35;
const float SECUREDISTANCE_FRONT = 0.15;
const int PARKINGSPEED = 4;


#endif /* PARK_H_ */
