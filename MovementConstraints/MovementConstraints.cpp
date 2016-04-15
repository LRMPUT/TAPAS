/*
	Copyright (c) 2014,	TAPAS Team:
	-Michal Nowicki (michal.nowicki@put.poznan.pl),
	-Jan Wietrzykowski (jan.wietrzykowski@cie.put.poznan.pl).
	Poznan University of Technology
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice,
	this list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice,
	this list of conditions and the following disclaimer in the documentation
	and/or other materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
	THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
	AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
	OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "MovementConstraints.h"

//#define MAP_SIZE 100	//100x100 MAP_STEPxMAP_STEP cells
//#define MAP_STEP 100	//in mm
//#define MAP_MARGIN 25	//25 cells margin

using namespace cv;
using namespace std;

MovementConstraints::MovementConstraints(TiXmlElement* settings) :
		runThread(false),
		constraintsMapThreadRunning(false),
		imuActive(false),
		encodersActive(false)
{
	if(!settings){
		throw "Bad settings file - entry MovementConstraints not found";
	}
	readSettings(settings);
	TiXmlElement* pCamera = settings->FirstChildElement("Camera");

	cameraConstraintsClient = nh.serviceClient<TAPAS::CameraConstraints>("camera_constraints");
	constraintsMap = Mat(MAP_SIZE, MAP_SIZE, CV_32FC1, Scalar(0));

	ros::ServiceServer service = nh.advertiseService("point_cloud", &MovementConstraints::getPointCloud, this);
	imu_sub = nh.subscribe("imu_data", 10, &MovementConstraints::imuCallback, this);
	encoders_sub = nh.subscribe("encoders_data", 10, &MovementConstraints::encodersCallback, this);

	//runThread = false;
	movementConstraintsThread = std::thread(&MovementConstraints::run, this);

}

MovementConstraints::~MovementConstraints() {
	cout << "~MovementConstraints()" << endl;
	stopThread();
	cout << "End ~MovementConstraints()" << endl;
}

void MovementConstraints::imuCallback(const TAPAS::IMU msg) {
	imuActive = true;
	std::unique_lock<std::mutex> lck(mtxImu);
	imuCur = RosHelpers::readIMUMsg(msg);
	lck.unlock();
}

void MovementConstraints::encodersCallback(const TAPAS::Encoders msg) {
	encodersActive = true;
	std::unique_lock<std::mutex> lck(mtxEncoders);
	encodersCur = Mat(2, 1, CV_32SC1);
	encodersCur.at<int>(0) = msg.left;
	encodersCur.at<int>(1) = msg.right;
	lck.unlock();
}

void MovementConstraints::readSettings(TiXmlElement* settings){
	if(settings->QueryBoolAttribute("runThread", &runThread) != TIXML_SUCCESS){
		throw "Bad settings file - no runThread setting for MovementConstraints";
	}
	if(settings->QueryIntAttribute("debug", &debugLevel) != TIXML_SUCCESS){
		throw "Bad settings file - no debug setting for MovementConstraints";
	}

	TiXmlElement* pOdom = settings->FirstChildElement("odometry");
	if(!pOdom){
		throw "Bad settings file - no odometry settings";
	}
	if(pOdom->QueryFloatAttribute("diam", &pointCloudSettings.wheelCir) != TIXML_SUCCESS){
		cout << "Warning - bad odometry diam";
	}
	pointCloudSettings.wheelCir *= PI;
	if(pOdom->QueryFloatAttribute("base", &pointCloudSettings.wheelDistance) != TIXML_SUCCESS){
		cout << "Warning - bad odometry base";
	}
	if(pOdom->QueryIntAttribute("encodersCPR", &pointCloudSettings.encodersCPR) != TIXML_SUCCESS){
		cout << "Warning - bad odometry encodersCPR";
	}

	TiXmlElement* pPtCloud = settings->FirstChildElement("point_cloud");
	if(!pPtCloud){
		throw "Bad settings file - no point_cloud settings";
	}
	if(pPtCloud->QueryIntAttribute("minDist", &pointCloudSettings.minLaserDist) != TIXML_SUCCESS){
		cout << "Warning - bad point_cloud minDist";
	}
	cout << "pointCloudSettings.minLaserDist = " <<  pointCloudSettings.minLaserDist << endl;

	if(pPtCloud->QueryIntAttribute("maxTimePoint", &pointCloudSettings.pointCloudTimeout) != TIXML_SUCCESS){
		cout << "Warning - bad point_cloud maxTimePoint";
	}
	if(pPtCloud->QueryIntAttribute("maxTimeMap", &pointCloudSettings.mapTimeout) != TIXML_SUCCESS){
		cout << "Warning - bad point_cloud maxTimeMap";
	}

	TiXmlElement* pLaserConstraints = settings->FirstChildElement("laser_constraints");
	if(!pLaserConstraints){
		throw "Bad settings file - no laser_constraints settings";
	}
	if(pLaserConstraints->QueryFloatAttribute("lowerThreshold", &laserLowerThreshold) != TIXML_SUCCESS){
		cout << "Warning - bad laser_constraints lowerThreshold";
	}
	if(pLaserConstraints->QueryFloatAttribute("upperThreshold", &laserUpperThreshold) != TIXML_SUCCESS){
		cout << "Warning - bad laser_constraints upperThreshold";
	}
	if(pLaserConstraints->QueryFloatAttribute("upperThreshold", &laserUpperThreshold) != TIXML_SUCCESS){
		cout << "Warning - bad laser_constraints upperThreshold";
	}
	if(pLaserConstraints->QueryIntAttribute("minPts", &laserMinPts) != TIXML_SUCCESS){
		cout << "Warning - bad laser_constraints minPts";
	}

	cameraOrigLaser = readMatrixSettings(settings, "camera_position_laser", 4, 4);
	cameraOrigImu = readMatrixSettings(settings, "imu_position_camera", 4, 4).t();
	imuOrigRobot = readMatrixSettings(settings, "imu_position_robot", 4, 4);
	//groundPlane = readMatrixSettings(settings, "ground_plane_global", 4, 1);
}

cv::Mat MovementConstraints::readMatrixSettings(TiXmlElement* parent, const char* node, int rows, int cols){
	TiXmlElement* ptr = parent->FirstChildElement(node);
	if(!ptr){
		throw (string("Bad settings file - no ") + string(node)).c_str();
	}
	//cout << "node: " << node << ", value: " << ptr->GetText() << endl;
	stringstream tmpStr(ptr->GetText());
	Mat ret = Mat(rows, cols, CV_32FC1);
	for(int row = 0; row < rows; row++){
		for(int col = 0; col < cols; col++){
			float tmpVal;
			tmpStr >> tmpVal;
			ret.at<float>(row, col) = tmpVal;
		}
	}
	return ret;
}

// Main loop of MovementContraints thread.
void MovementConstraints::run(){
	cout << "MovementConstraints::run()" << endl;
	try{
	#ifndef ROBOT_OFFLINE
		cout << "Waiting for encoders and IMU" << endl;
		while(runThread){
			if(encodersActive && imuActive){
				//cout << "robot->isEncodersOpen() = " << robot->isEncodersOpen() << ", robot->isImuOpen() = " << robot->isImuOpen() << endl;
				break;
			}
			//cout << "robot->isEncodersOpen() = " << robot->isEncodersOpen() << ", robot->isImuOpen() = " << robot->isImuOpen() << endl;
			std::chrono::milliseconds duration(100);
			std::this_thread::sleep_for(duration);
		}
	#endif

		int i = 0;
		while (runThread) {
			//cout << "Processing points cloud" << endl;
			//cout << "robot->isEncodersOpen() = " << robot->isEncodersOpen() << ", robot->isImuOpen() = " << robot->isImuOpen() << endl;
			//updateConstraintsMap(0, 0, 0);
			if(debugLevel >= 1){
				cout << "updatePointCloud()" << endl;
			}
			updatePointCloud();
			if(debugLevel >= 1){
				cout << "end updatePointCloud()" << endl;
			}

			if(i >= 25){
				if(debugLevel >= 1){
					cout << "updateConstraintsMap()" << endl;
				}
				if(!constraintsMapThreadRunning){
					if(debugLevel >= 1){
						cout << "joining updateConstraintsMap thread" << endl;
					}
					if(constraintsMapThread.joinable()){
						constraintsMapThread.join();
					}
					if(debugLevel >= 1){
						cout << "spawning new thread for updateConstraintsMap" << endl;
					}
					constraintsMapThreadRunning = true;
					constraintsMapThread = std::thread(&MovementConstraints::updateConstraintsMap, this);
				}
				if(debugLevel >= 1){
					cout << "end updateConstraintsMap()" << endl;
				}
				i = 0;
			}
			//20 ms sleep
			std::chrono::milliseconds duration(20);
			std::this_thread::sleep_for(duration);
			i++;
		}

		if(constraintsMapThread.joinable()){
			constraintsMapThread.join();
		}
	}
	catch(char const* error){
		cout << "Char exception in MovementConstraints: " << error << endl;
		exit(1);
	}
	catch(std::exception& e){
		cout << "Std exception in MovementConstraints: " << e.what() << endl;
		exit(1);
	}
	catch(...){
		cout << "Unexpected exception in MovementConstraints" << endl;
		exit(1);
	}
}

// Stop MovementConstraints thread.
void MovementConstraints::stopThread(){
	runThread = false;
	if(movementConstraintsThread.joinable()){
		movementConstraintsThread.join();
	}
}

void MovementConstraints::updateConstraintsMap(){
	// cout << "updateConstraintsMap()" << endl;

	//przesuwanie mapy
	std::chrono::high_resolution_clock::time_point timestampMapCur = std::chrono::high_resolution_clock::now();

//	cout << "Moving map" << endl;
	std::chrono::high_resolution_clock::time_point imuTimestamp;
//	cout << "updating cur pos" << endl;
	updateCurPosOrigMapCenter();

	std::unique_lock<std::mutex> lckMap(mtxMap);
	std::unique_lock<std::mutex> lckPointCloud(mtxPointCloud);
	std::unique_lock<std::mutex> lckImu(mtxImu);

	Mat mapMove = curPosOrigMapCenter.inv();

//	cout << "Calculating new curMapCenterOrigGlobal" << endl;
	curMapCenterOrigGlobal = ConstraintsHelpers::compOrient(imuCur);
	curPosOrigMapCenter = Mat::eye(4, 4, CV_32FC1);
//	cout << "Map moved" << endl;
	timestampMap = timestampMapCur;
	lckImu.unlock();
	lckPointCloud.unlock();
	lckMap.unlock();

	lckMap.lock();
	constraintsMap = Scalar(0);

	//polling each constraints module to update map
//	cout << "Adding constraints" << endl;
	this->insertHokuyoConstraints(constraintsMap, timestampMap, mapMove);
	// cout << "Adding camera constraints" << endl;
	insertCameraConstraints(constraintsMap, timestampMap, mapMove);
//	cout << constraintsMap << endl;
	lckMap.unlock();
	// cout << "End updateConstraintsMap()" << endl;

	constraintsMapThreadRunning = false;
}


void MovementConstraints::insertHokuyoConstraints(	cv::Mat map,
													std::chrono::high_resolution_clock::time_point curTimestampMap,
													cv::Mat mapMove)
{
//	cout << "insertHokuyoConstraints()" << endl;
	std::unique_lock<std::mutex> lckPointCloud(mtxPointCloud);
//	cout << "pointCloudOrigMapCenter.size() = " << pointCloudOrigMapCenter.size() << endl;
//	cout << "imuOrigRobot.size() = " << imuOrigRobot.size() << endl;
	if(!pointCloudOrigMapCenter.empty()){
		//Move points to new map center
		Mat newPointCloudCoords = mapMove*pointCloudOrigMapCenter.rowRange(0, 4);
		newPointCloudCoords.copyTo(pointCloudOrigMapCenter.rowRange(0, 4));
	}

	Mat pointCloudOrigRobotMapCenter;
	if(!pointCloudOrigMapCenter.empty()){
		pointCloudOrigRobotMapCenter = imuOrigRobot*pointCloudOrigMapCenter.rowRange(0, 4);
	}

	lckPointCloud.unlock();

//	cout << "pointCloudOrigRobotMapCenter.size() = " << pointCloudOrigRobotMapCenter.size() << endl;
	vector<vector<vector<Point3f> > > bins(MAP_SIZE, vector<vector<Point3f> >(MAP_SIZE, vector<Point3f>()));
	for(int p = 0; p < pointCloudOrigRobotMapCenter.cols; p++){
		int x = pointCloudOrigRobotMapCenter.at<float>(0, p)/MAP_RASTER_SIZE + MAP_SIZE/2;
		int y = pointCloudOrigRobotMapCenter.at<float>(1, p)/MAP_RASTER_SIZE + MAP_SIZE/2;
		if(x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE){
			//cout << "(" << x << ", " << y << ")" << endl;
			bins[x][y].push_back(Point3f(pointCloudOrigRobotMapCenter.at<float>(0, p),
											pointCloudOrigRobotMapCenter.at<float>(1, p),
											pointCloudOrigRobotMapCenter.at<float>(2, p)));
		}
	}
//	cout << "inserting into map" << endl;
	for(int y = 0; y < MAP_SIZE; y++){
		for(int x = 0; x < MAP_SIZE; x++){
			//Point3f minPoint;
			int count = 0;
			float sumVal = 0;
			for(int p = 0; p < bins[x][y].size(); p++){
				float val = -bins[x][y][p].z - laserLowerThreshold;
				val = min(max(val/(laserUpperThreshold - laserLowerThreshold), 0.0f), 1.0f);
				if(val > 0){
					count++;
				}
				sumVal += val;
				//minPoint = bins[x][y][p];
			}
			if(count >= laserMinPts){ //at least 5 points
				map.at<float>(x, y) = max(map.at<float>(x, y), sumVal/count);
				//cout << "Constraint at (" << x << ", " << y << ")" << endl;
				//cout << "Point = " << minPoint << endl;
			}
		}
	}
	//cout << "End insertHokuyoConstraints()" << endl;
}

void MovementConstraints::insertCameraConstraints(cv::Mat map,
								std::chrono::high_resolution_clock::time_point curTimestampMap,
								cv::Mat mapMove) {
	TAPAS::CameraConstraints srv;
	int map_size = MAP_SIZE*MAP_SIZE*sizeof(float);

	memcpy(srv.request.constraintsMap.data(), map.data, map_size);
	srv.request.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(curTimestampMap.time_since_epoch()).count();
	memcpy(srv.request.mapMove.data(), mapMove.data, 16*sizeof(float));
	cameraConstraintsClient.call(srv);
	memcpy(map.data, srv.response.constraintsMap.data(), map_size);
}

void MovementConstraints::updateCurPosOrigMapCenter(){
#ifndef ROBOT_OFFLINE
	if(imuActive && encodersActive){
		std::chrono::high_resolution_clock::time_point imuTimestamp;
		std::chrono::high_resolution_clock::time_point encTimestamp;

//		cout << "Euler angles: " << imuCur.at<float>(2, 3) << " " << imuCur.at<float>(1, 3) << " " << imuCur.at<float>(0, 3) << endl;
//		cout << "imuCur = " << imuCur << endl;
//		cout << "encodersCur = " << encodersCur << endl;

		std::unique_lock<std::mutex> lckImu(mtxImu);
		std::unique_lock<std::mutex> lckEncoders(mtxEncoders);
		if(imuPrev.empty()){
			imuCur.copyTo(imuPrev);
		}
		if(encodersPrev.empty()){
			encodersCur.copyTo(encodersPrev);
		}

		std::unique_lock<std::mutex> lck(mtxPointCloud);
		if(curMapCenterOrigGlobal.empty()){
			curMapCenterOrigGlobal = compOrient(imuCur);
		}
		if(curPosOrigMapCenter.empty()){
			curPosOrigMapCenter = Mat::eye(4, 4, CV_32FC1);
		}
		curPosOrigMapCenter = ConstraintsHelpers::compNewPos(imuPrev, imuCur,
											encodersPrev, encodersCur,
											curPosOrigMapCenter,
											curMapCenterOrigGlobal,
											pointCloudSettings);
		lck.unlock();
		//cout << "trans = " << trans << endl;
		//cout << "curMapCenterOrigGlobal = " << curMapCenterOrigGlobal << endl;
		//cout << "curPosOrigMapCenter = " << curPosOrigMapCenter << endl;
		//cout << "curTrans = " << curTrans << endl;
		//cout << "curRot = " << curRot << endl;
		//cout << "imuPosGlobal.inv()*curPos*cameraOrigImu.inv() = " << endl << imuPosGlobal.inv()*curPos*cameraOrigImu.front().inv() << endl;
		//cout << "globalPos.inv()*curPos = " << globalPos.inv()*curPos << endl;

		imuCur.copyTo(imuPrev);
		encodersCur.copyTo(encodersPrev);
		lckImu.unlock();
		lckEncoders.unlock();
	}
#else
	std::unique_lock<std::mutex> lck(mtxPointCloud);
//	cout << "updating curPosOrigMapCenter" << endl;
	curPosOrigMapCenter = Mat::eye(4, 4, CV_32FC1);
//	cout << "end updating curPosOrigMapCenter" << endl;
	lck.unlock();
#endif
}

void MovementConstraints::updatePointCloud(){
	//cout << "processPointCloud()" << endl;

	if(debugLevel >= 1){
		cout << "updatecurPosOrigMapCenter()" << endl;
	}
	updateCurPosOrigMapCenter();
	if(debugLevel >= 1){
		cout << "end updatecurPosOrigMapCenter()" << endl;
	}

	if(hokuyo.isDataValid()){
		std::chrono::high_resolution_clock::time_point hokuyoTimestamp;
		Mat hokuyoData = hokuyo.getData(hokuyoTimestamp);
		ConstraintsHelpers::processPointCloud(hokuyoData,
						pointCloudOrigMapCenter,
						pointsQueue,
						hokuyoTimestamp,
						std::chrono::high_resolution_clock::now(),
						curPosOrigMapCenter,
						mtxPointCloud,
						cameraOrigLaser,
						cameraOrigImu,
						nh);
	}
	else{
		if(debugLevel >= 1){
			cout << "Hokuyo closed" << endl;
		}
	}

	//cout << "End processPointCloud()" << endl;
}

const MovementConstraints::PointCloudSettings& MovementConstraints::getPointCloudSettings(){
	return pointCloudSettings;
}

//----------------------EXTERNAL ACCESS TO MEASUREMENTS
//CV_32SC1 4xHOKUYO_SCANS: x, y, distance, intensity - points from left to right
const cv::Mat MovementConstraints::getHokuyoData(){
	std::chrono::high_resolution_clock::time_point hokuyoTimestamp;
	return hokuyo.getData(hokuyoTimestamp);
}

//----------------------ACCESS TO COMPUTED DATA
//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied
const cv::Mat MovementConstraints::getMovementConstraints(){
	Mat ret;
	std::unique_lock<std::mutex> lck(mtxMap);
	constraintsMap.copyTo(ret);
	lck.unlock();
	return ret;
}

bool MovementConstraints::getPointCloud(TAPAS::PointCloud::Request &req, TAPAS::PointCloud::Response &res){
	//cout << "getPointCloud()" << endl;
	std::unique_lock<std::mutex> lck(mtxPointCloud);
	memcpy(res.curPosOrigMapCenter.data(), curPosOrigMapCenter.data, 16*sizeof(float));
	res.cloudRows = pointCloudOrigMapCenter.rows;
	res.cloudCols = pointCloudOrigMapCenter.cols;
	res.pointCloudOrigMapCenter.assign((float*)pointCloudOrigMapCenter.datastart, (float*)pointCloudOrigMapCenter.dataend);
	lck.unlock();
	//cout << "End getPointCloud()" << endl;
	return true;
}

cv::Mat MovementConstraints::getPointCloud(cv::Mat& curPosMapCenter){
	//cout << "getPointCloud()" << endl;
	Mat ret;
	std::unique_lock<std::mutex> lck(mtxPointCloud);
	pointCloudOrigMapCenter.copyTo(ret);
	if(!curPosOrigMapCenter.empty()){
		curPosOrigMapCenter.copyTo(curPosMapCenter);
	}
	lck.unlock();
	//cout << "End getPointCloud()" << endl;
	return ret;
}

cv::Mat MovementConstraints::getCurMapCenterOrigGlobal(){
	Mat ret;
	std::unique_lock<std::mutex> lck(mtxPointCloud);
	if(!curMapCenterOrigGlobal.empty()){
		curMapCenterOrigGlobal.copyTo(ret);
	}
	lck.unlock();
	return ret;
}

cv::Mat MovementConstraints::getPosImuMapCenter(){
	Mat ret;
	std::unique_lock<std::mutex> lck(mtxPointCloud);
	curPosOrigMapCenter.copyTo(ret);
	lck.unlock();
	return ret;
}

void MovementConstraints::getLocalPlanningData(cv::Mat& retConstraintsMap,cv::Mat& posRobotMapCenter, cv::Mat& globalMapCenter){

	std::unique_lock<std::mutex> lckMap(mtxMap);
	std::unique_lock<std::mutex> lckPC(mtxPointCloud);
	constraintsMap.copyTo(retConstraintsMap);
	Mat retPosRobotMapCenter;
	if(!curPosOrigMapCenter.empty() && !imuOrigRobot.empty()){
		retPosRobotMapCenter = curPosOrigMapCenter*imuOrigRobot;
	}
	posRobotMapCenter = retPosRobotMapCenter;
	//curPosOrigMapCenter.copyTo(posImuMapCenter);
	curMapCenterOrigGlobal.copyTo(globalMapCenter);
	lckPC.unlock();
	lckMap.unlock();

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
