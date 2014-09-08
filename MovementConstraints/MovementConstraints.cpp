/*
 * MovementConstaints.cpp
 *
 */

#include "MovementConstraints.h"

//#define MAP_SIZE 100	//100x100 MAP_STEPxMAP_STEP cells
//#define MAP_STEP 100	//in mm
//#define MAP_MARGIN 25	//25 cells margin

using namespace cv;
using namespace std;

MovementConstraints::MovementConstraints(Robot* irobot, TiXmlElement* settings) : robot(irobot) {
	if(!settings){
		throw "Bad settings file - entry MovementConstraints not found";
	}
	readSettings(settings);
	TiXmlElement* pCamera = settings->FirstChildElement("Camera");
	camera = new Camera(this, pCamera);

	constraintsMap = Mat(MAP_SIZE, MAP_SIZE, CV_32FC1, Scalar(0));

	runThread = true;
	movementConstraintsThread = std::thread(&MovementConstraints::run, this);

}

MovementConstraints::~MovementConstraints() {
	cout << "~MovementConstraints()" << endl;
	stopThread();
	delete camera;
	cout << "End ~MovementConstraints()" << endl;
}

void MovementConstraints::readSettings(TiXmlElement* settings){
	cameraOrigLaser = readMatrixSettings(settings, "camera_position_laser", 4, 4);
	cameraOrigImu = readMatrixSettings(settings, "imu_position_camera", 4, 4).t();
	imuOrigGlobal = readMatrixSettings(settings, "imu_position_global", 4, 4);
	groundPlane = readMatrixSettings(settings, "ground_plane_global", 4, 1);
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

#ifndef ROBOT_OFFLINE
	while(runThread){
		if(robot->isEncodersOpen() && robot->isImuOpen()){
			break;
		}
		std::chrono::milliseconds duration(100);
		std::this_thread::sleep_for(duration);
	}
#endif

	//1000 ms sleep
	std::chrono::milliseconds duration(1000);
	std::this_thread::sleep_for(duration);

	int i = 0;
	while (runThread) {
		//cout << "Processing points cloud" << endl;
		//updateConstraintsMap(0, 0, 0);
		updatePointCloud();

		if(i >= 25){
			updateConstraintsMap();
			i = 0;
		}
		//500 ms sleep
		std::chrono::milliseconds duration(20);
		std::this_thread::sleep_for(duration);
		i++;
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
	//cout << "updateConstraintsMap()" << endl;
	std::unique_lock<std::mutex> lckPointCloud(mtxPointCloud);
	double curMapX = curPosCloudMapCenter.at<float>(0, 3);
	double curMapY = curPosCloudMapCenter.at<float>(1, 3);
	lckPointCloud.unlock();

	//przesuwanie mapy
	bool move = false;
	if(curMapX > (MAP_SIZE/2 - MAP_MARGIN) * MAP_RASTER_SIZE){
		move = true;
	}
	else if(curMapX < (-MAP_SIZE/2 + MAP_MARGIN) * MAP_RASTER_SIZE){
		move = true;
	}
	if(curMapY > (MAP_SIZE/2 - MAP_MARGIN) * MAP_RASTER_SIZE){
		move = true;
	}
	else if(curMapY < (-MAP_SIZE/2 + MAP_MARGIN) * MAP_RASTER_SIZE){
		move = true;
	}
	static const float mapTimeout = 5000;
	std::chrono::high_resolution_clock::time_point timestampMapCur = std::chrono::high_resolution_clock::now();
	if(std::chrono::duration_cast<std::chrono::milliseconds>(timestampMapCur - timestampMap).count() > 5000){
		move = true;
	}
	if(move == true){
		//cout << "Moving map" << endl;
		std::chrono::high_resolution_clock::time_point imuTimestamp;
		Mat imuCur = robot->getImuData(imuTimestamp);
		//cout << "updating cur pos" << endl;
		updateCurPosCloudMapCenter();

		lckPointCloud.lock();
		std::unique_lock<std::mutex> lckMap(mtxMap);
		//cout << "locked" << endl;
		//cout << "Calculating new coords" << endl;
		if(!pointCloudImuMapCenter.empty()){
			//Move points to new map center
			Mat newPointCloudCoords = curPosCloudMapCenter.inv()*pointCloudImuMapCenter.rowRange(0, 4);
			newPointCloudCoords.copyTo(pointCloudImuMapCenter.rowRange(0, 4));
		}

		//cout << "Calculating new posMapCenterGlobal" << endl;
		posMapCenterGlobal = compOrient(imuCur);
		curPosCloudMapCenter = Mat::eye(4, 4, CV_32FC1);
		//cout << "Map moved" << endl;
		timestampMap = timestampMapCur;
		lckMap.unlock();
		lckPointCloud.unlock();
	}
	std::unique_lock<std::mutex> lckMap(mtxMap);
	constraintsMap = Scalar(0);

	//polling each constraints module to update map
	//this->insertHokuyoConstraints(constraintsMap);
	camera->insertConstraints(constraintsMap, timestampMap);
	//cout << constraintsMap << endl;
	lckMap.unlock();
	//cout << "End updateConstraintsMap()" << endl;
}


void MovementConstraints::insertHokuyoConstraints(cv::Mat map,
													std::chrono::high_resolution_clock::time_point curTimestampMap)
{
	//cout << "insertHokuyoConstraints()" << endl;
	std::unique_lock<std::mutex> lckPointCloud(mtxPointCloud);
	//Mat pointCloudImu = pointCloudImuMapCenter.rowRange(0, 4);
	vector<vector<vector<Point3f> > > bins(MAP_SIZE, vector<vector<Point3f> >(MAP_SIZE, vector<Point3f>()));
	for(int p = 0; p < pointCloudImuMapCenter.cols; p++){
		int x = pointCloudImuMapCenter.at<float>(0, p)/MAP_RASTER_SIZE + MAP_SIZE/2;
		int y = pointCloudImuMapCenter.at<float>(1, p)/MAP_RASTER_SIZE + MAP_SIZE/2;
		if(x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE){
			//cout << "(" << x << ", " << y << ")" << endl;
			bins[x][y].push_back(Point3f(pointCloudImuMapCenter.at<float>(0, p),
										pointCloudImuMapCenter.at<float>(1, p),
										pointCloudImuMapCenter.at<float>(2, p)));
		}
	}
	lckPointCloud.unlock();
	//cout << "inserting into map" << endl;
	for(int y = 0; y < MAP_SIZE; y++){
		for(int x = 0; x < MAP_SIZE; x++){
			static float heightThres = 730;
			static int numThres = 5;
			//Point3f minPoint;
			int count = 0;
			for(int p = 0; p < bins[x][y].size(); p++){
				if(bins[x][y][p].z <= heightThres){	//if something is taller than camera position minus 730 mm
					count++;
				}
				//minPoint = bins[x][y][p];
			}
			if(count >= numThres){ //at least 5 points
				map.at<float>(x, y) = 1;
				//cout << "Constraint at (" << x << ", " << y << ")" << endl;
				//cout << "Point = " << minPoint << endl;
			}
		}
	}
	//cout << "End insertHokuyoConstraints()" << endl;
}

void MovementConstraints::updateCurPosCloudMapCenter(){
#ifndef ROBOT_OFFLINE
	if(robot->isImuOpen() && robot->isEncodersOpen()){
		std::chrono::high_resolution_clock::time_point imuTimestamp;
		Mat encodersCur = robot->getEncoderData();
		Mat imuCur = robot->getImuData(imuTimestamp);

		//cout << "Euler angles: " << imuCur.at<float>(2, 3) << " " << imuCur.at<float>(1, 3) << " " << imuCur.at<float>(0, 3) << endl;
		//cout << "imuCur = " << imuCur << endl;
		//cout << "encodersCur = " << encodersCur << endl;

		if(imuPrev.empty()){
			imuCur.copyTo(imuPrev);
		}
		if(encodersPrev.empty()){
			encodersCur.copyTo(encodersPrev);
		}

		std::unique_lock<std::mutex> lck(mtxPointCloud);
		if(posMapCenterGlobal.empty()){
			posMapCenterGlobal = compOrient(imuCur);
		}
		if(curPosCloudMapCenter.empty()){
			curPosCloudMapCenter = Mat::eye(4, 4, CV_32FC1);
		}
		curPosCloudMapCenter = compNewPos(imuPrev, imuCur,
											encodersPrev, encodersCur,
											curPosCloudMapCenter,
											posMapCenterGlobal);
		lck.unlock();
		//cout << "trans = " << trans << endl;
		//cout << "posMapCenterGlobal = " << posMapCenterGlobal << endl;
		//cout << "curPosCloudMapCenter = " << curPosCloudMapCenter << endl;
		//cout << "curTrans = " << curTrans << endl;
		//cout << "curRot = " << curRot << endl;
		//cout << "imuPosGlobal.inv()*curPos*cameraOrigImu.inv() = " << endl << imuPosGlobal.inv()*curPos*cameraOrigImu.front().inv() << endl;
		//cout << "globalPos.inv()*curPos = " << globalPos.inv()*curPos << endl;

		imuCur.copyTo(imuPrev);
		encodersCur.copyTo(encodersPrev);
	}
#else
	std::unique_lock<std::mutex> lck(mtxPointCloud);
	curPosCloudMapCenter = Mat::eye(4, 4, CV_32FC1);
	lck.unlock();
#endif
}

void MovementConstraints::updatePointCloud(){
	//cout << "processPointCloud()" << endl;

	updateCurPosCloudMapCenter();

	if(hokuyo.isOpen()){
		std::chrono::high_resolution_clock::time_point hokuyoTimestamp;
		Mat hokuyoData = hokuyo.getData(hokuyoTimestamp);
		processPointCloud(hokuyoData,
						pointCloudImuMapCenter,
						pointsQueue,
						hokuyoTimestamp,
						std::chrono::high_resolution_clock::now(),
						curPosCloudMapCenter,
						mtxPointCloud,
						cameraOrigLaser,
						cameraOrigImu);
	}
	else{
		//cout << "Hokuyo closed" << endl;
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
							cv::Mat encodersDiff)
{
	static const float wheelCir = 178*PI;
	static const float wheelDistance = 432;
	static const int encodersCPR = 300;
	float sl = (float)encodersDiff.at<int>(0)*wheelCir/encodersCPR;
	float sr = (float)encodersDiff.at<int>(1)*wheelCir/encodersCPR;
	float theta = (sl - sr)/(-wheelDistance);
	Mat trans(4, 1, CV_32FC1, Scalar(0));
	//cout << "theta = " << theta << endl;
	if(theta < 0.1){
		trans.at<float>(0) = (sl + sr)/2;
	}
	else{
		float r = -wheelDistance*(sl - sr)/(2*(sl - sr));
		trans.at<float>(0) = r*sin(theta);
		trans.at<float>(1) = r*(cos(theta) - 1);

	}
	//cout << "transC = " << trans << endl << "orientC = " << orient << endl;
	trans = orient*trans;
	return trans;
}

cv::Mat MovementConstraints::compNewPos(cv::Mat lprevImu, cv::Mat lcurImu,
									cv::Mat lprevEnc, cv::Mat lcurEnc,
									cv::Mat lposMapCenter,
									cv::Mat lmapCenterGlobal)
{
	Mat ret = Mat::eye(4, 4, CV_32FC1);
	if(!lposMapCenter.empty() && !lmapCenterGlobal.empty()){
		//cout << "compOrient(lcurImu) = " << compOrient(lcurImu) << endl;
		//cout << "lmapCenterGlobal = " << lmapCenterGlobal << endl;
		//cout << "Computing curPos" << endl;
		//cout << "encodersCur - encodersPrev = " << encodersCur - encodersPrev << endl;
		Mat trans = lmapCenterGlobal.inv()*compTrans(compOrient(lprevImu), lcurEnc - lprevEnc);
		//cout << "trans = " << trans << endl;
		//cout << "Computing curTrans" << endl;
		Mat curTrans = Mat(lposMapCenter, Rect(3, 0, 1, 4)) + trans;
		//cout << "Computing curRot" << endl;

		Mat curRot = lmapCenterGlobal.inv()*compOrient(lcurImu);
		//cout << "curRot = " << curRot << endl;

		curRot.copyTo(ret);
		curTrans.copyTo(Mat(ret, Rect(3, 0, 1, 4)));
	}
	return ret;
}

void MovementConstraints::processPointCloud(cv::Mat hokuyoData,
											cv::Mat& pointCloudImuMapCenter,
											std::queue<PointsPacket>& pointsInfo,
											std::chrono::high_resolution_clock::time_point hokuyoTimestamp,
											std::chrono::high_resolution_clock::time_point curTimestamp,
											cv::Mat curPosCloudMapCenter,
											std::mutex& mtxPointCloud,
											cv::Mat cameraOrigLaser,
											cv::Mat cameraOrigImu)
{
	Mat hokuyoCurPoints(6, hokuyoData.cols, CV_32FC1);
	//cout << "Copying hokuyo data" << endl;
	int countPoints = 0;
	for(int c = 0; c < hokuyoData.cols; c++){
		//cout << hokuyoData.at<int>(2, c) << endl;
		if(hokuyoData.at<int>(2, c) > 500){
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
	//cout << "countPoints = " << countPoints << ", hokuyoCurPoints.size = " << hokuyoCurPoints.size() << endl;
	if(countPoints > 0){

		//cout << "Moving to camera orig" << endl;
		hokuyoCurPoints.rowRange(0, 4) = cameraOrigLaser.inv()*hokuyoCurPoints.rowRange(0, 4);

		//cout << "Removing old points" << endl;
		static const int timeoutThres = 5000;
		//remove all points older than 2000 ms
		int pointsSkipped = 0;
		if(pointsInfo.size() > 0){
			//cout << "dt = " << std::chrono::duration_cast<std::chrono::milliseconds>(curTimestamp - pointsQueue.front().timestamp).count() << endl;
			while(std::chrono::duration_cast<std::chrono::milliseconds>(curTimestamp - pointsInfo.front().timestamp).count() > timeoutThres){
				pointsSkipped += pointsInfo.front().numPoints;
				pointsInfo.pop();
				if(pointsInfo.size() == 0){
					break;
				}
			}
		}

		//cout << "Moving pointCloudImuMapCenter, pointsSkipped = " << pointsSkipped << endl;
		Mat tmpAllPoints(hokuyoCurPoints.rows, pointCloudImuMapCenter.cols + hokuyoCurPoints.cols - pointsSkipped, CV_32FC1);
		if(!pointCloudImuMapCenter.empty()){
			//cout << "copyTo" << endl;
			//TODO check copying when all points are being removed
			pointCloudImuMapCenter.colRange(pointsSkipped,
												pointCloudImuMapCenter.cols).
							copyTo(tmpAllPoints.colRange(0, pointCloudImuMapCenter.cols - pointsSkipped));
		}
		//cout << "Addding hokuyoCurPoints" << endl;
		if(std::chrono::duration_cast<std::chrono::milliseconds>(curTimestamp - hokuyoTimestamp).count() <= timeoutThres){
			Mat curPointCloudCameraMapCenter(hokuyoCurPoints.rows, hokuyoCurPoints.cols, CV_32FC1);
			Mat tmpCurPoints = curPosCloudMapCenter*cameraOrigImu*hokuyoCurPoints.rowRange(0, 4);
			tmpCurPoints.copyTo(curPointCloudCameraMapCenter.rowRange(0, 4));
			hokuyoCurPoints.rowRange(4, 6).copyTo(curPointCloudCameraMapCenter.rowRange(4, 6));
			//cout << hokuyoCurPointsGlobal.channels() << ", " << hokuyoAllPointsGlobal.channels() << endl;
			curPointCloudCameraMapCenter.copyTo(tmpAllPoints.colRange(pointCloudImuMapCenter.cols - pointsSkipped,
																		pointCloudImuMapCenter.cols + hokuyoCurPoints.cols - pointsSkipped));

			pointsInfo.push(PointsPacket(hokuyoTimestamp, hokuyoCurPoints.cols));
		}
		std::unique_lock<std::mutex> lck(mtxPointCloud);
		pointCloudImuMapCenter = tmpAllPoints;
		lck.unlock();
	}
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
	pointCloudImuMapCenter.copyTo(ret);
	if(!curPosCloudMapCenter.empty()){
		curPosMapCenter = curPosCloudMapCenter;
	}
	lck.unlock();
	//cout << "End getPointCloud()" << endl;
	return ret;
}

cv::Mat MovementConstraints::getPosImuMapCenter(){
	Mat ret;
	std::unique_lock<std::mutex> lck(mtxPointCloud);
	curPosCloudMapCenter.copyTo(ret);
	lck.unlock();
	return ret;
}

void MovementConstraints::getLocalPlanningData(cv::Mat& MovementConstraints,cv::Mat& PosImuMapCenter, cv::Mat& GlobalMapCenter){

	std::unique_lock<std::mutex> lckMap(mtxMap);
	std::unique_lock<std::mutex> lckPC(mtxPointCloud);
	constraintsMap.copyTo(MovementConstraints);
	curPosCloudMapCenter.copyTo(PosImuMapCenter);
	posMapCenterGlobal.copyTo(GlobalMapCenter);
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
