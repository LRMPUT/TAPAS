/*
 * IMU_driver.h
 *
 *  Created on: Sep 1, 2014
 *      Author: smi
 */

#ifndef IMU_DRIVER_H_
#define IMU_DRIVER_H_

#include <ctime>
#include <string>
#include <chrono>
#include <stdio.h>
#include <unistd.h>
#include <thread>

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
	float* getGyro();
	float* getAccel();
	float* getMag();
	float* getEuler();

	// Set data from gyro, accelerometer, magnetometer and euler angles.
	void setGyro(float* gyro);
	void setAccel(float* acc);
	void setMag(float* mag);
	void setEuler(float roll, float pitch, float yaw);

	// ping device
	void pingDevice();

	// Get information about the IMU
	void printInformation();

	// get Timestamp of last orientation update
	std::chrono::high_resolution_clock::time_point getTimestamp();
	void setTimestamp(std::chrono::high_resolution_clock::time_point _timestamp);

	// Port openings
	void openPort(std::string& device);

private:
	//! Serial port baud length
	static const int Baud = 115200;

	// Microstrain class
	mip_interface device_interface;

	// Thread reading info from sensor
	std::thread processingThread;
	bool runThread;

	// Current data
	float acc[3], gyro[3], mag[3], euler[3];

	// Timestamp of last measurement
	std::chrono::high_resolution_clock::time_point timestamp;

	// method called in another thread
	void updateIMU();
};

#endif /* IMU_DRIVER_H_ */
