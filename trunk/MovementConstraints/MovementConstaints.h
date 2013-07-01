/*
 * MovementConstaints.h
 *
 */

#ifndef MOVEMENTCONSTAINTS_H_
#define MOVEMENTCONSTAINTS_H_

//#include "Camera/***"
#include "Hokuyo/Hokuyo.h"
#include "Sharp/Sharp.h"
#include <vector>
#include <opencv2/opencv.hpp>


class MovementConstaints {

	// Class to get data from Camera

	// Class to get data from Hokuyo
	Hokuyo hokuyo;

	// Class to get data from Sharp
	Sharp sharp;

public:
	MovementConstaints();
	virtual ~MovementConstaints();

	//x, y points
	void getHokuyoData(std::vector<int>& data);

	//left, right image
	void getCameraData(std::vector<cv::Mat>& data);
};

#endif /* MOVEMENTCONSTAINTS_H_ */
