/*
 * PositionEstimation.h
 *
 * In this class, we integrate data from different sensors using Kalman Filter
 *
 * TO THINK: - find Kalman implementation and maybe put it in another class
 */

#ifndef POSITIONESTIMATION_H_
#define POSITIONESTIMATION_H_

#include <vector>
#include "Encoders/Encoders.h"
#include "GPS/GPS.h"
#include "IMU/IMU.h"

class PositionEstimation {

private:
	// Encoders
	Encoders encoders;

	// GPS
	GPS gps;

	// IMU
	IMU imu;

public:
	PositionEstimation();
	virtual ~PositionEstimation();

	// Update Kalman - updates on GPS
	void KalmanUpdate();

	// Encoders - predict
	void KalmanPredict();

	//left, right encoder
	void getEncoderData(std::vector<int>& data);

	//x, y position
	void getGpsData(std::vector<int>& data);

	//acc, gyro, magnet, euler
	void getImuData(std::vector<float>& data);
};

#endif /* POSITIONESTIMATION_H_ */
