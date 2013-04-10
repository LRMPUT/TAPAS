/*
 * MovementConstaints.h
 *
 */

#ifndef MOVEMENTCONSTAINTS_H_
#define MOVEMENTCONSTAINTS_H_

//#include "Camera/***"
#include "Hokuyo/Hokuyo.h"
#include "Sharp/Sharp.h"

class MovementConstaints {

	// Class to get data from Camera

	// Class to get data from Hokuyo
	Hokuyo hokuyo;

	// Class to get data from Sharp
	Sharp sharp;

public:
	MovementConstaints();
	virtual ~MovementConstaints();
};

#endif /* MOVEMENTCONSTAINTS_H_ */
