/*
 * MechanicalGrabber.h
 *
 * TO DO: - create simple interface allowing us to grab gold ;p
 *
 */

#ifndef MECHANICALGRABBER_H_
#define MECHANICALGRABBER_H_

class MechanicalGrabber {
private:

	bool active;

public:
	MechanicalGrabber();
	virtual ~MechanicalGrabber();

	// Grab an object - true if successful
	bool grab();

	// Ungrab an object - true is successful
	bool ungrab();

	// Method informs if the grabber is performing some action (we can't move) - true is grabber is active
	bool isActive();

};

#endif /* MECHANICALGRABBER_H_ */
