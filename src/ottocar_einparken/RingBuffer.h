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

class RingBuffer
{
public:
	RingBuffer();
	virtual ~RingBuffer();

	void insert (float value);
	float getMedian();
	void printBuffer();

private:
	float buffer[5];
	int bufferSize;
	int bufferPointer;
};

#endif /* RINGBUFFER_H_ */
