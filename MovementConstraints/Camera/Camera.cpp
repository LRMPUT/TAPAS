/*
 * Camera.cpp
 *
 *  Created on: 08-07-2013
 *      Author: jachu
 */

#include <opencv2/opencv.hpp>
#include "Camera.h"

using namespace cv;
using namespace std;

Camera::Camera(MovementConstraints* imovementConstraints) : movementConstraints(imovementConstraints) {

}

Camera::~Camera(){

}

//CV_8UC3 2x640x480: left, right image
cv::Mat Camera::getData(){
	//empty matrix
	int size[] = {2, 640, 480};
	return Mat(3, size, CV_8UC3);
}

void Camera::open(){

}

void Camera::close(){

}

bool Camera::isOpen(){
	return true;
}
