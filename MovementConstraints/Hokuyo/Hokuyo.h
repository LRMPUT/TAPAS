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

#ifndef HOKUYO_H_
#define HOKUYO_H_

class Hokuyo;

#include <string>
#include <thread>
#include <mutex>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <urg_cpp/Urg_driver.h>

#define HOKUYO_SCANS 1081

class Debug;

class Hokuyo {
	friend class Debug;

	enum class HokuyoType{
		URG04LX,
		UTM30LX
	};

	std::thread readingThread;
	volatile bool runThread;

	qrk::Urg_driver hokuyo;

	HokuyoType hokuyoType;

	cv::Mat curMeas;
	std::chrono::high_resolution_clock::time_point curTimestamp;
	std::mutex mtx;

	bool dataValid;

	void run();
public:
	Hokuyo();
	virtual ~Hokuyo();

	void openPort(std::string port);

	void closePort();

	bool isOpen();

	bool isDataValid();
	//CV_32SC1 4xHOKUYO_SCANS: x, y, distance, intensity - points from left to right
	cv::Mat getData(std::chrono::high_resolution_clock::time_point &timestamp);
};


#endif /* HOKUYO_H_ */
