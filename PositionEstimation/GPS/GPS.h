/*
 * GPS.h
 *
 * TO DO: 	- Create a way of getting to GPS data
 * 			- Recalculate global position (longitude etc.) to (x,y,z)
 * TO THINK:- if we can measure a map before start, how should we remeber
 * 			 matching between global and local coordinates for start and gold ?
 */

#ifndef GPS_H_
#define GPS_H_

class GPS {
public:
	GPS();
	virtual ~GPS();
};

#endif /* GPS_H_ */
