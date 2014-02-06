/*
 * RingBuffer.cpp
 *
 *  Created on: 03.02.2014
 *      Author: licht
 */

#include "RingBuffer.h"

RingBuffer::RingBuffer()
{
	bufferSize = 5;
	bufferPointer = 0;

	for (int i = 0; i < bufferSize; i++)
	{
		buffer[i] = 0;
	}
}

RingBuffer::~RingBuffer()
{

}

void RingBuffer::insert (float value)
{
	buffer[bufferPointer] = value;
	bufferPointer++;

	if (bufferPointer >= bufferSize)
	{
		bufferPointer = 0;
	}

	//todo nur für Testzwecke! sonst bitte auskommentieren
	if (abs(value - getMedian()) > 0.05)
	{
		ROS_WARN("[BUF]: neuer Wert stark abweichend: %2.4f | %2.4f", value, getMedian());
	}
}


float RingBuffer::getMedian()
{
	float copieBuffer[bufferSize];
	for (int i = 0; i < bufferSize; i++)
	{
		copieBuffer[i] = buffer[i];
	}
	std::sort(copieBuffer, copieBuffer + bufferSize);
	return copieBuffer[(bufferSize / 2)];
}

void RingBuffer::printBuffer()
{
	ROS_INFO("[BUF]: ----------------");
	for (int i = 0; i < bufferSize; i++)
	{
		ROS_INFO("[BUF]: buffer[%d]: %2.4f", i, buffer[i]);
	}
}



