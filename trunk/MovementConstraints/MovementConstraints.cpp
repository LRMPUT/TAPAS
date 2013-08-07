/*
 * MovementConstaints.cpp
 *
 */

#include "MovementConstraints.h"

using namespace cv;
using namespace std;

MovementConstraints::MovementConstraints(Robot* irobot) : robot(irobot) {
	camera = new Camera(this, NULL);
}

MovementConstraints::~MovementConstraints() {
	delete camera;
}

//----------------------ACCESS TO COMPUTED DATA
//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robot's position (MAP_SIZE/2, 0)
const cv::Mat MovementConstraints::getMovementConstraints(){
	return Mat(MAP_SIZE, MAP_SIZE, CV_32FC1);
}

//----------------------EXTERNAL ACCESS TO MEASUREMENTS
//CV_32SC1 2xHOKUYO_SCANS: x, y points from left to right
const cv::Mat MovementConstraints::getHokuyoData(){
	return hokuyo.getData();
}

//CV_8UC3 2x640x480: left, right image
const std::vector<cv::Mat> MovementConstraints::getCameraData(){
	return camera->getData();
}

//----------------------MENAGMENT OF MovementConstraints DEVICES
//Hokuyo
void MovementConstraints::openHokuyo(std::string port){
	hokuyo.openPort(port);
}

void MovementConstraints::closeHokuyo(){
	hokuyo.closePort();
}

bool MovementConstraints::isHokuyoOpen(){
	return hokuyo.isOpen();
}

//Camera
void MovementConstraints::openCamera(){
	camera->open();
}

void MovementConstraints::closeCamera(){
	camera->close();
}

bool MovementConstraints::isCameraOpen(){
	return camera->isOpen();
}
