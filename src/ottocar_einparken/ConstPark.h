/*
* ConstPark.h
*
* Created on: Dec 6, 2013
*
*/

#ifndef PARK_H_
#define PARK_H_



///Geschwindigkeit, mit der das Programm ausgeführt werden soll
const int LOOP_RATE = 50;

//ParkingConst
/**\brief set the "nan" and "infinity" value of laser scanner data to avoid calculator failure **/
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
const float LARGEGAP = 0.8;
/**\brief parking by sequence **/
const bool SEQUENCEPARK = true;
/**\brief parking in fist gap**/
const bool FIRSTPARK=true;

/*
* DriveIntoGap
*/
///Wert für den Servo, um geradeaus zu fahren
const int STRAIGHTFORWARD = 7;
///Maximalwert des Servos links
const int LEFT_MAX = -127;
///Maximalwert des Servos rechts
const int RIGHT_MAX = 126;

/*
* Fahrwerte
*/
///Impulse der Odometrie pro Meter
const float REVOLUTIONS_PER_M = 4500.0;
///Geschwindigkeit bis zur Positionierung vor der Lücke
const int SPEED_DRIVING = 23;
///Geschwindigkeit beim Einparken
const int SPEED_PARKING = 21;

/*
* ParallelController
*/
///Seitliche Entfernung des Autos zu den Kartons
const float TARGET_DISTANCE = 0.17;
///Wert zwischen 0 bis 511, bis zu dem die Daten des Laserscanners ausgewertet werden. 0 ist rechts
const int SEARCH_SPACE = 256;

#endif /* PARK_H_ */
