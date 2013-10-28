/*
 * debug.cpp
 *
 *  Created on: 04-10-2013
 *      Author: jachu
 */

#include "Debug.h"
//OpenCV
#include <opencv2/opencv.hpp>
//Boost
#include <boost/filesystem.hpp>
//RobotsIntellect
#include "../Robot/Robot.h"
#include "../MovementConstraints/MovementConstraints.h"
#include "../MovementConstraints/Camera/Camera.h"
#include "../MovementConstraints/Hokuyo/Hokuyo.h"
#include "../Planning/GlobalPlanner.h"
#include "../PositionEstimation/PositionEstimation.h"
#include "../PositionEstimation/GPS/GPS.h"
#include "../PositionEstimation/IMU/IMU.h"

using namespace cv;
using namespace std;
using namespace boost;

Debug::Debug(Robot* irobot) : robot(irobot){

}

//----------------------MODES OF OPERATION
void Debug::switchMode(OperationMode mode){
	robot->globalPlanner.switchMode(mode);
}

void Debug::setMotorsVel(float motLeft, float motRight){
	robot->globalPlanner.setMotorsVel(motLeft, motRight);
}

//----------------------EXTERNAL ACCESS TO MEASUREMENTS
//CV_32SC1 2x1: left, right encoder
const cv::Mat Debug::getEncoderData(){
	return robot->globalPlanner.getEncoderData();
}

//CV_32SC1 4x1: x, y, lat, lon position
const cv::Mat Debug::getGpsData(){
	Mat ret(4, 1, CV_32FC1);
	ret.at<float>(0) = robot->positionEstimation.gps.getPosX();
	ret.at<float>(1) = robot->positionEstimation.gps.getPosY();
	ret.at<float>(2) = robot->positionEstimation.gps.getLat();
	ret.at<float>(3) = robot->positionEstimation.gps.getLon();
	return ret;
}

//1 - no fix, 2 - 2D, 3 - 3D
int Debug::getGpsFixStatus(){
	return robot->positionEstimation.gps.getFixStatus();
}

int Debug::getGpsSatelitesUsed(){
	return robot->positionEstimation.gps.getSatelitesUsed();
}

void Debug::setGpsZeroPoint(double lat, double lon){
	robot->positionEstimation.gps.setZeroXY(lat, lon);
}

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(yaw, pitch, roll)
const cv::Mat Debug::getImuData(){
	return robot->positionEstimation.imu.getData();
}

//CV_32SC1 2xHOKUYO_SCANS: x, y points from left to right
const cv::Mat Debug::getHokuyoData(){
	return robot->movementConstraints->hokuyo.getData();
}

//CV_8UC3 2x640x480: left, right image
const std::vector<cv::Mat> Debug::getCameraData(){
	return robot->movementConstraints->camera->getData();
}

void Debug::testSegmentation(boost::filesystem::path dir){
	filesystem::directory_iterator endIt;
	namedWindow("segmented");
	namedWindow("original");
	for(filesystem::directory_iterator dirIt(dir); dirIt != endIt; dirIt++){
		cout << dirIt->path().string() << endl;
		if(dirIt->path().filename().string().find(".jpg") != string::npos){
			cout << "Processing image " << dirIt->path().string() << endl;
			Mat image = imread(dirIt->path().string());
			imshow("original", image);
			robot->movementConstraints->camera->segment(image);
			waitKey(1000);
			char tmpChar;
			cin >> tmpChar;
		}
	}
}

//----------------------ACCESS TO COMPUTED DATA
//CV_32SC1 3x1: x, y, fi
const cv::Mat Debug::getEstimatedPosition(){
	return robot->positionEstimation.getEstimatedPosition();
}

//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robot's position (MAP_SIZE/2, 0)
const cv::Mat Debug::getMovementConstraints(){
	return robot->movementConstraints->getMovementConstraints();
}