/*
 * Kinect.h
 *
 *  Simple interface to get data and return it in some way
 */

#ifndef KINECT_H_
#define KINECT_H_

class Kinect;

#include <string>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>

class Debug;

class Kinect {
	friend class Debug;

	std::thread readingThread;
	volatile bool runThread;


	cv::Mat curMeas;
	std::mutex mtx;

	void run();
public:
	Kinect();
	virtual ~Kinect();

	void openPort(std::string port);

	void closePort();

	bool isOpen();

	//CV_
	const cv::Mat getData();
};


#endif /* KINECT_H_ */
