/*
 * Camera.h
 *
 *  Created on: 01-07-2013
 *      Author: jachu
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <opencv2/opencv.hpp>

class MovementConstraints;

class Camera {
	MovementConstraints* movementConstraints;
public:
	Camera(MovementConstraints* imovementConstraints);
	virtual ~Camera();

	//CV_8UC3 2x640x480: left, right image
	cv::Mat getData();

	void open();

	void close();

	bool isOpen();
};

#include "../MovementConstraints.h"

#endif /* CAMERA_H_ */
