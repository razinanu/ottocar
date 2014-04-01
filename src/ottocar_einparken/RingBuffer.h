/*
 * RingBuffer.h
 *
 *  Created on: 03.02.2014
 *      Author: licht
 */

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include "ros/ros.h"
#include "ConstPark.h"

 /**
  * \brief  Ringpuffer der Kapazität 5
  *
  *         Diese Klasse stellt die Funktionalität eines Ringpuffers mit der Kapazität
  *         von 5 Elementen vom Typ Float zur Verfügung.
  *
  */

class RingBuffer
{
public:
	RingBuffer();
	virtual ~RingBuffer();

	 /**
	  * \brief  Element einfügen
	  *
	  *         Diese Funktion ermöglicht das Einfügen eines Floatwertes in den
	  *         Ringpuffer.
	  *
	  * \param	value   einzufügender Wert vom Typ Float
	  *
	  */
	void insert (float value);

	 /**
	  * \brief  Median bestimmen
	  *
	  *         Diese Funktion ermittelt den Median aller Werte des Ringpuffers
	  *         und gibt diesen zurück.
	  *
	  * \return 	Median vom Typ Float
	  *
	  */
	float getMedian();

	///Diese Funktion gibt den gesamten Inhalt des Ringpuffers auf der Konsole aus.
	void printBuffer();

private:
	float buffer[5];
	int bufferSize;
	int bufferPointer;
};

#endif /* RINGBUFFER_H_ */
