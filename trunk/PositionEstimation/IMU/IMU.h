/*
 * IMU.h
 *
 * Not necessarily needed - we'll see during the project development - sMi
 */

#ifndef IMU_H_
#define IMU_H_

#include <opencv2/opencv.hpp>
#include "../../Trobot/include/Imu.h"
#include "include/IMU_driver.h"

class Debug;
class Robot;

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

class IMU {
	friend class Debug;

	trobot::Imu* imu;
	Robot *robot;

public:
	IMU();
	IMU(Robot* irobot);
	virtual ~IMU();

	void openPort(std::string port);
	void closePort();
	bool isPortOpen();

	//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(roll, pitch, yaw)
	cv::Mat getData(std::chrono::high_resolution_clock::time_point &timestamp);
};

#endif /* IMU_H_ */
