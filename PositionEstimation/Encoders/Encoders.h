/*
	Copyright (c) 2014,	TAPAS Team:
	-Michal Nowicki (michal.nowicki@put.poznan.pl),
	-Jan Wietrzykowski (jan.wietrzykowski@cie.put.poznan.pl).
	Poznan University of Technology
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice,
	this list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice,
	this list of conditions and the following disclaimer in the documentation
	and/or other materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
	THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
	AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
	OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ENCODERS_H_
#define ENCODERS_H_

#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "../../Trobot/include/SerialPort.h"
#include "ros/ros.h"
#include "TAPAS/Encoders.h"

class Robot;

class Encoders {
	trobot::SerialPort serialPort;
	Robot *robot;
	ros::NodeHandle nh;

	std::thread readingThread, dataThread;
	cv::Mat curEnc;
	std::chrono::high_resolution_clock::time_point curTimestamp;
	std::mutex mtx;
	bool runThread;
	bool dataValid;

	void sendData();
	void run();
public:
	Encoders();
	Encoders(Robot *irobot);
	//Encoders(const std::string& device, unsigned int baud);
	virtual ~Encoders();


	void openPort(const std::string& device, unsigned int baud);
	void closePort();
	bool isPortOpen();
	bool isDataValid();

	cv::Mat getEncoders(std::chrono::high_resolution_clock::time_point &timestamp);
};

#endif /* ENCODERS_H_ */
