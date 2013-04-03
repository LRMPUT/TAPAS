/*
 * MechanicalGrabber.cpp
 *
 */

#include "MechanicalGrabber.h"

MechanicalGrabber::MechanicalGrabber() {
	active = false;

}

MechanicalGrabber::~MechanicalGrabber() {
}

bool MechanicalGrabber::grab() {

	return true;
}

bool MechanicalGrabber::ungrab() {

	return true;
}

bool MechanicalGrabber::isActive() {

	return false;
}
