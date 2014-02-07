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
const float BIGRANGE = 99.0;
const float PARALLELDISTANCE = 0.17;

//GapCalculatorConst
/**\brief valid value for difference between base horizontal distance and second corner horizontal distance **/
const float MINDISTANCE = 0.1;
/**\brief distance of first obstacle **/
const float FIRSTDISTANCE = 0.2;
const float BESTGAPLENGTH = 0.6;
const float SMALLGAP = 0.6;
const float MEDIUMGAP = 0.7;
const float LARGGAP = 0.8;
/**\brief parking by sequence **/
const bool SEQUENCEPARK = true;
/**\brief parking in fist gap**/
const bool FIRSTPARK=true;

//Lenkwerte
const int STRAIGHTFORWARD =  4;
const int LEFT_MAX = -127;
const int RIGHT_MAX = 126;

//Fahrwerte
//const float REVOLUTIONS_PER_M = 1404.8;	//Laborfußboden
const float REVOLUTIONS_PER_M = 4500.0;
const int SPEED_PARKING = 19;	//todo 18 bei vollem Akku

/*
 * ParallelController
 */

const float TARGET_DISTANCE = 0.17; //Entfernung d, in der ein Karton stehen sollte
const float TOLERANCE = 1.3; //Toleranz; Abweichung darf r*t betragen
const float CONFIDENCE = 1.5; //Faktor fuer das Verhaeltnis von a und f
const float MAX_RANGE = 2.0; //Begrenzung des Sichtfeldes
const float MIN_RANGE = 0.2; //Begrenzung des Sichtfeldes
const int SEARCH_SPACE = 256; //Wert, bis zu dem die Laserscans ausgewertet werden

//DriveIntoGap
const float SECUREDISTANCE_BACK = 0.35;
const float SECUREDISTANCE_FRONT = 0.15;
const int PARKINGSPEED = 8;

#endif /* PARK_H_ */
