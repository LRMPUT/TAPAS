/*
 * IMU.h
 *
 * Not necessarily needed - we'll see during the project development - sMi
 */

#ifndef IMU_H_
#define IMU_H_

#include <opencv2/opencv.hpp>
#include "../../Trobot/include/Imu.h"

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
	trobot::Imu* imu;
public:
	IMU();
	virtual ~IMU();

	void openPort(std::string port);
	void closePort();
	bool isPortOpen();

	//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
	const cv::Mat getData();
};

#endif /* IMU_H_ */
