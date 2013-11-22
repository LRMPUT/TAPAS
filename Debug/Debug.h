/*
 * debug.h
 *
 *  Created on: 04-10-2013
 *      Author: jachu
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "../Robot/Robot.h"

class Debug{
	Robot* robot;
public:
	Debug(Robot* irobot);

	//----------------------MODES OF OPERATION
	void switchMode(OperationMode mode);

	void setMotorsVel(float motLeft, float motRight);

	//----------------------EXTERNAL ACCESS TO MEASUREMENTS
	//CV_32SC1 2x1: left, right encoder
	const cv::Mat getEncoderData();

	//CV_32SC1 4x1: x, y, lat, lon position
	const cv::Mat getGpsData();

	//1 - no fix, 2 - 2D, 3 - 3D
	int getGpsFixStatus();

	int getGpsSatelitesUsed();

	void setGpsZeroPoint(double lat, double lon);

	//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
	const cv::Mat getImuData();

	//CV_32SC1 4xHOKUYO_SCANS: x, y, distance, intensity - points from left to right
	const cv::Mat getHokuyoData();

	//CV_8UC3 2x640x480: left, right image
	const std::vector<cv::Mat> getCameraData();

	void testSegmentation(boost::filesystem::path dir);

	//----------------------ACCESS TO COMPUTED DATA
	//CV_32SC1 3x1: x, y, fi
	const cv::Mat getEstimatedPosition();

	//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robot's position (MAP_SIZE/2, 0)
	const cv::Mat getMovementConstraints();
};


#endif /* DEBUG_H_ */
