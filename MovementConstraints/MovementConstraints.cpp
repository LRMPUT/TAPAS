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

MovementConstraints::MovementConstraints(Robot* irobot, TiXmlElement* settings) :
		robot(irobot),
		runThread(false),
		constraintsMapThreadRunning(false)
{
	if(!settings){
		throw "Bad settings file - entry MovementConstraints not found";
	}
	readSettings(settings);
	TiXmlElement* pCamera = settings->FirstChildElement("Camera");
	camera = new Camera(this, pCamera);


	constraintsMap = Mat(MAP_SIZE, MAP_SIZE, CV_32FC1, Scalar(0));

	//runThread = false;
	movementConstraintsThread = std::thread(&MovementConstraints::run, this);

}

MovementConstraints::~MovementConstraints() {
	cout << "~MovementConstraints()" << endl;
	delete camera;
	stopThread();
	cout << "End ~MovementConstraints()" << endl;
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
		while(runThread){
			if(robot->isEncodersOpen() && robot->isImuOpen()){
				//cout << "robot->isEncodersOpen() = " << robot->isEncodersOpen() << ", robot->isImuOpen() = " << robot->isImuOpen() << endl;
				break;
			}
			//cout << "robot->isEncodersOpen() = " << robot->isEncodersOpen() << ", robot->isImuOpen() = " << robot->isImuOpen() << endl;
			std::chrono::milliseconds duration(100);
			std::this_thread::sleep_for(duration);
		}
		cout << "Waiting for isImuDataValid()" << endl;
		while(runThread && !robot->isImuDataValid()){
			//cout << "Waiting for isImuDataValid()" << endl;
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
//	cout << "updateConstraintsMap()" << endl;

	//przesuwanie mapy
	std::chrono::high_resolution_clock::time_point timestampMapCur = std::chrono::high_resolution_clock::now();

//	cout << "Moving map" << endl;
	std::chrono::high_resolution_clock::time_point imuTimestamp;
//	cout << "getting imu data" << endl;
	Mat imuCur(3, 4, CV_32FC1, Scalar(0));
	if(robot->isImuDataValid()){
		imuCur = robot->getImuData(imuTimestamp);
	}
//	cout << "updating cur pos" << endl;
	updateCurPosOrigMapCenter();

	std::unique_lock<std::mutex> lckMap(mtxMap);
	std::unique_lock<std::mutex> lckPointCloud(mtxPointCloud);

	Mat mapMove = curPosOrigMapCenter.inv();

//	cout << "Calculating new curMapCenterOrigGlobal" << endl;
	curMapCenterOrigGlobal = compOrient(imuCur);
	curPosOrigMapCenter = Mat::eye(4, 4, CV_32FC1);
//	cout << "Map moved" << endl;
	timestampMap = timestampMapCur;
	lckPointCloud.unlock();
	lckMap.unlock();

	lckMap.lock();
	constraintsMap = Scalar(0);

	//polling each constraints module to update map
//	cout << "Adding constraints" << endl;
	this->insertHokuyoConstraints(constraintsMap, timestampMap, mapMove);
//	cout << "Adding camera constraints" << endl;
	camera->insertConstraints(constraintsMap, timestampMap, mapMove);
//	cout << constraintsMap << endl;
	lckMap.unlock();
//	cout << "End updateConstraintsMap()" << endl;

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

void MovementConstraints::updateCurPosOrigMapCenter(){
#ifndef ROBOT_OFFLINE
	if(robot->isImuOpen() && robot->isEncodersOpen()){
		std::chrono::high_resolution_clock::time_point imuTimestamp;
		std::chrono::high_resolution_clock::time_point encTimestamp;
		Mat encodersCur = robot->getEncoderData(encTimestamp);
		Mat imuCur = robot->getImuData(imuTimestamp);

//		cout << "Euler angles: " << imuCur.at<float>(2, 3) << " " << imuCur.at<float>(1, 3) << " " << imuCur.at<float>(0, 3) << endl;
//		cout << "imuCur = " << imuCur << endl;
//		cout << "encodersCur = " << encodersCur << endl;

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
		curPosOrigMapCenter = compNewPos(imuPrev, imuCur,
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
		processPointCloud(hokuyoData,
						pointCloudOrigMapCenter,
						pointsQueue,
						hokuyoTimestamp,
						std::chrono::high_resolution_clock::now(),
						curPosOrigMapCenter,
						mtxPointCloud,
						cameraOrigLaser,
						cameraOrigImu,
						pointCloudSettings);
	}
	else{
		if(debugLevel >= 1){
			cout << "Hokuyo closed" << endl;
		}
	}

	//cout << "End processPointCloud()" << endl;
}

cv::Mat MovementConstraints::compOrient(cv::Mat imuData){
	//cout << "Computing orientation from IMU" << endl;
	//cout << "imuData = " << imuData << endl;

	Mat ret(Mat::eye(4, 4, CV_32FC1));
	float yaw = imuData.at<float>(2, 3)*PI/180;
	float pitch = imuData.at<float>(1, 3)*PI/180;
	float roll = imuData.at<float>(0, 3)*PI/180;
	//cout << "Computing Rz, Ry, Rx, yaw = " << yaw << endl;
	Matx33f Rz(	cos(yaw), -sin(yaw), 0,
				sin(yaw), cos(yaw), 0,
				0, 0, 1);
	//cout << "Rz = " << Rz << endl;
	Matx33f Ry(	cos(pitch), 0, sin(pitch),
				0, 1, 0,
				-sin(pitch), 0, cos(pitch));
	//cout << "Ry = " << Ry << endl;
	Matx33f Rx(	1, 0, 0,
				0, cos(roll), -sin(roll),
				0, sin(roll), cos(roll));
	//cout << "Rx = " << Rx << endl;
	Mat tmp(Rz*Ry*Rx);
	tmp.copyTo(ret(Rect(0, 0, 3, 3)));

	//cout << "End computing orientation from IMU" << endl;
	return ret;
}


cv::Mat MovementConstraints::compTrans(	cv::Mat orient,
							cv::Mat encodersDiff,
							const PointCloudSettings& pointCloudSettings)
{

	float sl = (float)encodersDiff.at<int>(0)*pointCloudSettings.wheelCir/pointCloudSettings.encodersCPR;
	float sr = (float)encodersDiff.at<int>(1)*pointCloudSettings.wheelCir/pointCloudSettings.encodersCPR;

	if(fabs(sl) > 10000 || fabs(sr) > 10000){
		sl = 0.0f;
		sr = 0.0f;
	}

	float theta = (sl - sr)/(-pointCloudSettings.wheelDistance);
	Mat trans(4, 1, CV_32FC1, Scalar(0));
	//cout << "theta = " << theta << endl;
	if(theta < 0.1){
		trans.at<float>(0) = (sl + sr)/2;
	}
	else{
		float r = -pointCloudSettings.wheelDistance*(sl - sr)/(2*(sl - sr));
		trans.at<float>(0) = r*sin(theta);
		trans.at<float>(1) = r*(cos(theta) - 1);

	}
	//cout << "transC = " << trans << endl << "orientC = " << orient << endl;
	trans = orient*trans;
	return trans;
}

cv::Mat MovementConstraints::compNewPos(cv::Mat lprevImu, cv::Mat lcurImu,
									cv::Mat lprevEnc, cv::Mat lcurEnc,
									cv::Mat lposOrigMapCenter,
									cv::Mat lmapCenterOrigGlobal,
									const PointCloudSettings& pointCloudSettings)
{
	Mat ret = Mat::eye(4, 4, CV_32FC1);
	if(!lposOrigMapCenter.empty() && !lmapCenterOrigGlobal.empty()){
		//cout << "compOrient(lcurImu) = " << compOrient(lcurImu) << endl;
		//cout << "lmapCenterGlobal = " << lmapCenterGlobal << endl;
		//cout << "Computing curPos" << endl;
		//cout << "encodersCur - encodersPrev = " << encodersCur - encodersPrev << endl;
		Mat trans = lmapCenterOrigGlobal.inv()*compTrans(compOrient(lprevImu), lcurEnc - lprevEnc, pointCloudSettings);
		//cout << "trans = " << trans << endl;
		//cout << "Computing curTrans" << endl;
		Mat curTrans = Mat(lposOrigMapCenter, Rect(3, 0, 1, 4)) + trans;
		//cout << "Computing curRot" << endl;

		Mat curRot = lmapCenterOrigGlobal.inv()*compOrient(lcurImu);
		//cout << "curRot = " << curRot << endl;

		curRot.copyTo(ret);
		curTrans.copyTo(Mat(ret, Rect(3, 0, 1, 4)));
	}
	return ret;
}

void MovementConstraints::processPointCloud(cv::Mat hokuyoData,
											cv::Mat& pointCloudOrigMapCenter,
											std::queue<PointsPacket>& pointsInfo,
											std::chrono::high_resolution_clock::time_point hokuyoTimestamp,
											std::chrono::high_resolution_clock::time_point curTimestamp,
											cv::Mat curPosOrigMapCenter,
											std::mutex& mtxPointCloud,
											cv::Mat cameraOrigLaser,
											cv::Mat cameraOrigImu,
											const PointCloudSettings& pointCloudSettings)
{
	Mat hokuyoCurPoints(6, hokuyoData.cols, CV_32FC1);
	//cout << "Copying hokuyo data" << endl;
	int countPoints = 0;
	for(int c = 0; c < hokuyoData.cols; c++){
		//cout << hokuyoData.at<int>(2, c) << endl;
		if(hokuyoData.at<int>(2, c) > pointCloudSettings.minLaserDist){
			hokuyoCurPoints.at<float>(0, countPoints) = -hokuyoData.at<int>(1, c);
			hokuyoCurPoints.at<float>(1, countPoints) = 0.0;
			hokuyoCurPoints.at<float>(2, countPoints) = hokuyoData.at<int>(0, c);
			hokuyoCurPoints.at<float>(3, countPoints) = 1.0;
			hokuyoCurPoints.at<float>(4, countPoints) = hokuyoData.at<int>(2, c);
			hokuyoCurPoints.at<float>(5, countPoints) = hokuyoData.at<int>(3, c);
			countPoints++;
		}
	}
	hokuyoCurPoints = hokuyoCurPoints.colRange(0, countPoints);
//	cout << "hokuyoCurPoints.size() = " << hokuyoCurPoints.size() << endl;
//	cout << "hokuyoData.size() = " << hokuyoData.size() << endl;

	//cout << "Removing old points" << endl;
	//remove all points older than pointCloudTimeout ms
	int pointsSkipped = 0;
	if(pointsInfo.size() > 0){
		//cout << "dt = " << std::chrono::duration_cast<std::chrono::milliseconds>(curTimestamp - pointsQueue.front().timestamp).count() << endl;
		while(std::chrono::duration_cast<std::chrono::milliseconds>(curTimestamp - pointsInfo.front().timestamp).count() > pointCloudSettings.pointCloudTimeout){
			pointsSkipped += pointsInfo.front().numPoints;
			pointsInfo.pop();
			if(pointsInfo.size() == 0){
				break;
			}
		}
	}

	std::unique_lock<std::mutex> lck(mtxPointCloud);
	//cout << "Moving pointCloudOrigMapCenter, pointsSkipped = " << pointsSkipped << endl;
	Mat tmpAllPoints(hokuyoCurPoints.rows, pointCloudOrigMapCenter.cols + hokuyoCurPoints.cols - pointsSkipped, CV_32FC1);
	if(!pointCloudOrigMapCenter.empty()){
		//cout << "copyTo" << endl;
		if(pointsSkipped <	pointCloudOrigMapCenter.cols){
			pointCloudOrigMapCenter.colRange(pointsSkipped,
												pointCloudOrigMapCenter.cols).
							copyTo(tmpAllPoints.colRange(0, pointCloudOrigMapCenter.cols - pointsSkipped));
		}
	}
//	if(debugLevel >= 1){
//		cout << "countPoints = " << countPoints << ", hokuyoCurPoints.size = " << hokuyoCurPoints.size() << endl;
//	}
	if(countPoints > 0){

		//cout << "Moving to camera orig" << endl;
		hokuyoCurPoints.rowRange(0, 4) = cameraOrigLaser.inv()*hokuyoCurPoints.rowRange(0, 4);
		//cout << "Addding hokuyoCurPoints" << endl;
//		if(debugLevel >= 1){
//			cout << "Addding hokuyoCurPoints" << endl;
//		}
		Mat curPointCloudOrigMapCenter(hokuyoCurPoints.rows, hokuyoCurPoints.cols, CV_32FC1);
		Mat tmpCurPoints = curPosOrigMapCenter*cameraOrigImu*hokuyoCurPoints.rowRange(0, 4);
		tmpCurPoints.copyTo(curPointCloudOrigMapCenter.rowRange(0, 4));
		hokuyoCurPoints.rowRange(4, 6).copyTo(curPointCloudOrigMapCenter.rowRange(4, 6));
		//cout << hokuyoCurPointsGlobal.channels() << ", " << hokuyoAllPointsGlobal.channels() << endl;
//		cout << pointCloudOrigMapCenter.size() << " " << curPointCloudCameraMapCenter.size() << " " << tmpAllPoints.size() << endl;
//		if(debugLevel >= 1){
//			cout << "pointsSkipped = " << pointsSkipped << endl;
//		}
		curPointCloudOrigMapCenter.copyTo(tmpAllPoints.colRange(pointCloudOrigMapCenter.cols - pointsSkipped,
																	pointCloudOrigMapCenter.cols + hokuyoCurPoints.cols - pointsSkipped));
//		if(debugLevel >= 1){
//			cout << "curPointCloudCameraMapCenter copied" << endl;
//		}
		pointsInfo.push(PointsPacket(hokuyoTimestamp, hokuyoCurPoints.cols));

		pointCloudOrigMapCenter = tmpAllPoints;
	}
	lck.unlock();
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

//Camera
void MovementConstraints::openCamera(std::vector<std::string> device){
	camera->open(device);
}

void MovementConstraints::closeCamera(){
	camera->close();
}

bool MovementConstraints::isCameraOpen(){
	return camera->isOpen();
}
