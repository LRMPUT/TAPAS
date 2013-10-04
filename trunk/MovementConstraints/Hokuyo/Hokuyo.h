/*
 * Hokuyo.h
 *
 *  Simple interface to get data and return it in some way
 */

#ifndef HOKUYO_H_
#define HOKUYO_H_

class Hokuyo;

#include <string>
#include <opencv2/opencv.hpp>
#include <urg_c/urg_sensor.h>

#define HOKUYO_SCANS 1440

class Debug;

class Hokuyo {
	friend class Debug;

	urg_t hokuyo;
public:
	Hokuyo();
	virtual ~Hokuyo();

	void openPort(std::string port);

	void closePort();

	bool isOpen();

	//CV_32SC1 2xHOKUYO_SCANS: x, y points from left to right
	const cv::Mat getData();
};


#endif /* HOKUYO_H_ */
