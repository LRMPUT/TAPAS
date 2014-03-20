/*
 * Encoders.h
 *
 * TO DO: use windows library and make it linux, so we could have encoders data
 * Probably needs async communication, so new thread to receive data ? (fork in linux)
 */

#ifndef ENCODERS_H_
#define ENCODERS_H_

#include <string>
#include <opencv2/opencv.hpp>
#include "../../Trobot/include/SerialPort.h"

class Encoders {
	trobot::SerialPort serialPort;
public:
	Encoders();
	Encoders(const std::string& device, unsigned int baud);
	virtual ~Encoders();


	void openPort(const std::string& device, unsigned int baud);
	void closePort();
	bool isPortOpen();
	cv::Mat getEncoders();
};

#endif /* ENCODERS_H_ */
