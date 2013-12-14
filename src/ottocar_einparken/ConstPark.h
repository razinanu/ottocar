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

const float DISTANCE_RIGHT = 0.145;
const float TOLEARNCE_DISTANCE_RIGHT = 0.01;
const float DISTANCE_NO_OBSTACLE_RIGHT = 0.25;

/*
 * ParallelController
 */
//Entfernung d, in der ein Karton stehen sollte
const float TARGET_DISTANCE = 0.2;

//Toleranz; Abweichung darf r*t betragen
const float TOLERANCE = 2.0;

//Faktor fuer das Verhaeltnis von a und f
const float CONFIDENCE = 1.5;

//Wert, bis zu dem die Laserscans ausgewertet werden
const int SEARCH_SPACE = 256;

//DriveIntoGap
const float SECUREDISTANCE_BACK = 0.35;
const float SECUREDISTANCE_FRONT = 0.15;
const int PARKINGSPEED = 4;

#endif /* PARK_H_ */
