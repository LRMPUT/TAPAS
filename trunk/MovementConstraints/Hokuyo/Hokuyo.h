/*
 * Hokuyo.h
 *
 *  Simple interface to get data and return it in some way
 */

#ifndef HOKUYO_H_
#define HOKUYO_H_

class Hokuyo;

#include <string>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <urg_cpp/Urg_driver.h>

#define HOKUYO_SCANS 1081

class Debug;

class Hokuyo {
	friend class Debug;

	std::thread readingThread;
	volatile bool runThread;

	qrk::Urg_driver hokuyo;

	cv::Mat curMeas;
	std::mutex mtx;

	void run();
public:
	Hokuyo();
	virtual ~Hokuyo();

	void openPort(std::string port);

	void closePort();

	bool isOpen();

	//CV_32SC1 4xHOKUYO_SCANS: x, y, distance, intensity - points from left to right
	const cv::Mat getData();
};


#endif /* HOKUYO_H_ */
