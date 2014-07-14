/*
 * debug.cpp
 *
 *  Created on: 04-10-2013
 *      Author: jachu
 */

#include "Debug.h"
//STL
#include <chrono>
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
	std::chrono::high_resolution_clock::time_point timestamp;
	return robot->positionEstimation.getEncoderData(timestamp);
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

//CV_32FC1 3x4: acc(x, y, z), gyro(x, y, z), magnet(x, y, z), euler(roll, pitch, yaw)
const cv::Mat Debug::getImuData(){
	std::chrono::high_resolution_clock::time_point timestamp;
	return robot->positionEstimation.imu.getData(timestamp);
}

//CV_32SC1 4xHOKUYO_SCANS: x, y, distance, intensity - points from left to right
const cv::Mat Debug::getHokuyoData(){
	std::chrono::high_resolution_clock::time_point timestamp;
	return robot->movementConstraints->hokuyo.getData(timestamp);
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
			if(robot->movementConstraints->camera->hierClassifiers.size() > 0){
				Mat segments = robot->movementConstraints->camera->hierClassifiers[0]->segmentImage(image);
				Mat colored = robot->movementConstraints->camera->hierClassifiers[0]->colorSegments(segments);
				imshow("original", image);
				imshow("segmented", colored);
			}
			waitKey();
		}
	}
}

void Debug::testTraining(boost::filesystem::path dir){
	robot->movementConstraints->camera->learnFromDir(dir);
}

void Debug::testClassification(	boost::filesystem::path dirTrain,
								boost::filesystem::path dirTest)
{
	robot->movementConstraints->camera->learnFromDir(dirTrain);
	robot->movementConstraints->camera->classifyFromDir(dirTest);
}

void Debug::testConstraints(boost::filesystem::path dirTrain,
							boost::filesystem::path dirTest)
{
	//robot->movementConstraints->camera->learnFromDir(dirTrain);
	robot->movementConstraints->camera->readCache("cameraCache");
	filesystem::directory_iterator endIt;
	for(filesystem::directory_iterator dirIt(dirTrain); dirIt != endIt; dirIt++){
		if(dirIt->path().filename().string().find(".jpg") != string::npos){
			Mat image = imread(dirIt->path().string());
			if(image.data == NULL){
				throw "Bad image file";
			}
			vector<Mat> data(1, image);
			robot->movementConstraints->camera->computeConstraints(data);
			waitKey();
		}
	}
}

void Debug::testEncoders(){
	robot->openEncoders("/dev/robots/encoders");
	while(true){

		cout << robot->getEncoderData() << endl;
		char a = waitKey(500);
		if(a == 'q'){
			break;
		}
	}
}

void Debug::testDrivers(){
	robot->openRobotsDrive("/dev/robots/driver1", "/dev/robots/driver2");

	while(true){

		robot->globalPlanner.setMotorsVel(1000, 1000);
		char a = waitKey(500);
		if(a == 'q'){
			break;
		}
		robot->globalPlanner.setMotorsVel(0, 0);
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

std::vector<cv::Point2f> Debug::getPointCloudCamera(cv::Mat& image){
	Mat curPosMapCenter;
	Mat pointCloudCameraMapCenter = robot->movementConstraints->getPointCloud(curPosMapCenter);
	//cout << "Got point cloud" << endl;
	if(robot->movementConstraints->camera->isOpen()){
		image = robot->movementConstraints->camera->getData().front();
	}
	vector<Point2f> pointsImage;
	//cout << curPosMapCenter.size() << ", " << pointCloudCameraMapCenter.size() << endl;
	if(!curPosMapCenter.empty() && !pointCloudCameraMapCenter.empty()){
		Mat cameraOrigImu = robot->movementConstraints->cameraOrigImu;
		Mat allPointsCamera = (curPosMapCenter*cameraOrigImu).inv()*pointCloudCameraMapCenter.rowRange(0, 4);
		//cout << "Computing point projection" << endl;

		projectPoints(	allPointsCamera.rowRange(0, 3).t(),
						Matx<float, 3, 1>(0, 0, 0),
						Matx<float, 3, 1>(0, 0, 0),
						robot->movementConstraints->camera->cameraMatrix.front(),
						robot->movementConstraints->camera->distCoeffs.front(),
						pointsImage);
	}
	return pointsImage;
}


cv::Mat Debug::getPointCloudImu(cv::Mat& curPosImuMapCenter){
	Mat pointCloudImuMapCenter = robot->movementConstraints->getPointCloud(curPosImuMapCenter);
	return pointCloudImuMapCenter;
}

void Debug::getTransformationMatrices(cv::Mat& retImuOrigGlobal,
									cv::Mat& retCameraOrigLaser,
									cv::Mat& retCameraOrigImu)
{
	robot->movementConstraints->imuOrigGlobal.copyTo(retImuOrigGlobal);
	robot->movementConstraints->cameraOrigLaser.copyTo(retCameraOrigLaser);
	robot->movementConstraints->cameraOrigImu.copyTo(retCameraOrigImu);
}
