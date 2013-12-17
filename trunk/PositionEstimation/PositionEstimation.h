/*
 * PositionEstimation.h
 *
 * In this class, we integrate data from different sensors using Kalman Filter
 *
 * TO THINK: - find Kalman implementation and maybe put it in another class
 */

#ifndef POSITIONESTIMATION_H_
#define POSITIONESTIMATION_H_

#include <opencv2/opencv.hpp>
#include "Encoders/Encoders.h"
#include "GPS/GPS.h"
#include "IMU/IMU.h"
#include <thread>


class Robot;
class Debug;

class PositionEstimation {
	friend class Debug;

private:
	// unique pointer to the PositionEstimation thread
	std::unique_ptr<std::thread> pe_thread;

	// Kalman filter to gather position information
	cv::Mat state;
	cv::KalmanFilter *KF;

	// GPS
	GPS gps;

	// IMU
	IMU imu;

	// Encoders
	Encoders encoders;

	//Parent class Robot
	Robot* robot;

	const int ENCODER_TICK_PER_REV = 48 * 75;
	const double WHEEL_DIAMETER = 0.12;
	const double WHEEL_BASE = 0.24;



public:
	PositionEstimation(Robot* irobot);
	virtual ~PositionEstimation();

	// The cycle of the position estimation thread
	void run();

	// Update Kalman - updates on GPS
	void KalmanUpdate();

	// Encoders - predict
	void KalmanPredict();

	//----------------------ACCESS TO COMPUTED DATA
	//CV_32SC1 3x1: x, y, fi
	const cv::Mat getEstimatedPosition();

	//----------------------MENAGMENT OF PositionEstimation DEVICES
	//Gps
	void openGps(std::string port);

	void closeGps();

	bool isGpsOpen();

	//Imu
	void openImu(std::string port);

	void closeImu();

	bool isImuOpen();
};

#include "../Robot/Robot.h"

#endif /* POSITIONESTIMATION_H_ */
