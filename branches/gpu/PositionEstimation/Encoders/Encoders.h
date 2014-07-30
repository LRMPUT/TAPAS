/*
 * Encoders.h
 *
 * TO DO: use windows library and make it linux, so we could have encoders data
 * Probably needs async communication, so new thread to receive data ? (fork in linux)
 */

#ifndef ENCODERS_H_
#define ENCODERS_H_

#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "../../Trobot/include/SerialPort.h"

class Robot;

class Encoders {
	trobot::SerialPort serialPort;
	Robot *robot;

	std::thread readingThread;
	cv::Mat curEnc;
	std::chrono::high_resolution_clock::time_point curTimestamp;
	std::mutex mtx;
	bool runThread;

	void run();
public:
	Encoders();
	Encoders(Robot *irobot);
	//Encoders(const std::string& device, unsigned int baud);
	virtual ~Encoders();


	void openPort(const std::string& device, unsigned int baud);
	void closePort();
	bool isPortOpen();
	cv::Mat getEncoders(std::chrono::high_resolution_clock::time_point &timestamp);
};

#endif /* ENCODERS_H_ */
