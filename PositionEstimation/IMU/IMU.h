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

#ifndef IMU_H_
#define IMU_H_

#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#ifdef TROBOT
#include "../../Trobot/include/Imu.h"
#endif
#include "include/IMU_driver.h"
#include "ros/ros.h"
#include "TAPAS/IMU.h"
#include "../../Robot/RosHelpers.h"

class Debug;
class Robot;


#ifdef TROBOT
#define NUM_VALUES 12

struct Quantity {
	Quantity();
	Quantity(int iaddr, float ifactor){
		address = iaddr;
		factor = ifactor;
	}
	int address;
	float factor;
};
#endif

class IMU {

	enum IMU_TYPE {IMU_UM6, IMU_MICROSTRAIN_GX4_25};

	IMU_TYPE usedIMUType;

	friend class Debug;

	IMU_driver* imuNew;
#ifdef TROBOT
	trobot::Imu* imu;
#endif
	Robot *robot;
	ros::NodeHandle nh;

	std::thread dataThread;

	cv::Mat getUM6Data(std::chrono::high_resolution_clock::time_point &timestamp);
	cv::Mat getGX4Data(std::chrono::high_resolution_clock::time_point &timestamp);

public:
	IMU();
	IMU(Robot* irobot);
	virtual ~IMU();

	void sendData();
	void openPort(std::string port);
	void closePort();
	bool isPortOpen();
	bool isDataValid();
	float getAccVariance();

	//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(roll, pitch, yaw)
	cv::Mat getData(std::chrono::high_resolution_clock::time_point &timestamp);


};

#endif /* IMU_H_ */
