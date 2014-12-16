/*Copyright (c) 2014, TAPAS Team (cybair [at] put.poznan.pl), Poznan University of Technology
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
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

#ifndef IMU_DRIVER_H_
#define IMU_DRIVER_H_

#include <ctime>
#include <string>
#include <chrono>
#include <stdio.h>
#include <unistd.h>
#include <thread>
#include <list>
#include <mutex>

// Microstrain dependencies
extern "C" {
	#include "mip_sdk.h"
	#include "byteswap_utilities.h"
	#include "mip_gx4_imu.h"
	#include "mip_gx4_25.h"
}


#define DEFAULT_PACKET_TIMEOUT_MS  1000

#define MIP_SDK_GX4_25_IMU_STANDARD_MODE  0x01
#define MIP_SDK_GX4_25_IMU_DIRECT_MODE	  0x02

//#define IMU_DRIVER_DEBUG

class IMU_driver {
public:
	IMU_driver();
	~IMU_driver();

	// Read data from gyro, accelerometer, magnetometer and euler angles.
	void getGyro(float* gyroValues);
	void getAccel(float* accValues);
	void getMag(float* magValues);
	void getEuler(float* eulerValues);

	// Set data from gyro, accelerometer, magnetometer and euler angles.
	void setGyro(float* gyro);
	void setAccel(float* acc);
	void setMag(float* mag);
	void setEuler(float roll, float pitch, float yaw);

	// Get the variance of the accelerometer
	float getAccVariance();

	// ping device
	void pingDevice();

	// Get information about the IMU
	void printInformation();

	// get Timestamp of last orientation update
	std::chrono::high_resolution_clock::time_point getTimestamp();
	void setTimestamp(std::chrono::high_resolution_clock::time_point _timestamp);

	// Port openings
	void openPort(std::string& device);

	bool isDataValid();

private:
	//! Serial port baud length
	static const int Baud = 115200;

	// Microstrain class
	mip_interface device_interface;

	// Thread reading info from sensor
	std::thread processingThread;
	bool runThread;

	// Current data
	std::mutex accMtx, gyroMtx, magMtx, eulerMtx;
	float acc[3], gyro[3], mag[3], euler[3];

	// hsitory to compute variance
	std::mutex accHistoryMtx;
	std::list<float> accHistory;

	bool accValid, gyroValid, magValid, eulerValid;
	// Timestamp of last measurement
	std::chrono::high_resolution_clock::time_point timestamp;

	// method called in another thread
	void updateIMU();
};

#endif /* IMU_DRIVER_H_ */
