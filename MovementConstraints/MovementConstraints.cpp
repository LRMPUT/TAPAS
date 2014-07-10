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
	delete camera;
}

void MovementConstraints::readSettings(TiXmlElement* settings){
	cameraOrigLaser = readMatrixSettings(settings, "camera_position_laser", 4, 4);
	cameraOrigImu = readMatrixSettings(settings, "imu_position_camera", 4, 4).t();
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
	//500 ms sleep
	std::chrono::milliseconds duration(1000);
	std::this_thread::sleep_for(duration);

	while (runThread) {
		//cout << "Processing points cloud" << endl;
		//updateConstraintsMap(0, 0, 0);
		processPointCloud();

		//500 ms sleep
		std::chrono::milliseconds duration(20);
		std::this_thread::sleep_for(duration);
	}
}

// Stop MovementConstraints thread.
void MovementConstraints::stopThread(){
	runThread = false;
	movementConstraintsThread.join();
}

void MovementConstraints::updateConstraintsMap(double curGlobX, double curGlobY, double curGlobPhi){
	double curMapX = curGlobX - mapCenterX;
	double curMapY = curGlobY - mapCenterY;
	double curMapPhi = curGlobPhi - mapCenterPhi;

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
	if(move == true){
		std::unique_lock<std::mutex> lck(mtxPointCloud);
		mapCenterX = curGlobX;
		mapCenterY = curGlobY;
		mapCenterPhi = curGlobPhi;

		std::chrono::high_resolution_clock::time_point imuTimestamp;
		Mat imuCur = robot->getImuData(imuTimestamp);
		updateCurPosCloudMapCenter();

		//Move points to new map center
		Mat newPointCloudCoords = curPosCloudMapCenter.inv()*pointCloudCameraMapCenter.rowRange(0, 4);
		newPointCloudCoords.copyTo(pointCloudCameraMapCenter.rowRange(0, 4));

		posMapCenterGlobal = compOrient(imuCur);
		curPosCloudMapCenter = Mat::eye(4, 4, CV_32FC1);
		lck.unlock();
	}
	std::unique_lock<std::mutex> lck(mtxMap);
	constraintsMap = Scalar(0);

	//polling each constraints module to update map
	insertHokuyoConstraints(constraintsMap, curGlobX - mapCenterX, curGlobY - mapCenterY, curGlobPhi - mapCenterPhi);
	lck.unlock();
}


void MovementConstraints::insertHokuyoConstraints(cv::Mat map, double curMapX, double curMapY, double curMapPhi){
	Mat pointCloudImu = cameraOrigImu.inv()*pointCloudCameraMapCenter.rowRange(0, 4);
	vector<vector<vector<Point3f> > > bins(MAP_SIZE, vector<vector<Point3f> >(MAP_SIZE, vector<Point3f>()));
	for(int p = 0; p < pointCloudImu.cols; p++){
		int x = pointCloudImu.at<float>(0, p)/MAP_RASTER_SIZE;
		int y = pointCloudImu.at<float>(1, p)/MAP_RASTER_SIZE;
		bins[x][y].push_back(Point3f(pointCloudImu.at<float>(0, p),
									pointCloudImu.at<float>(1, p),
									pointCloudImu.at<float>(2, p)));
	}
	for(int y = 0; y < MAP_SIZE; y++){
		for(int x = 0; x < MAP_SIZE; y++){
			float minZ = 10e9;
			for(int p = 0; p < bins[x][y].size(); p++){
				minZ = min(minZ, bins[x][y][p].z);
			}
			if(minZ < 700){	//if something is taller than camera position minus 700 mm
				map.at<float>(x, y) = 1;
			}
		}
	}
}

cv::Mat MovementConstraints::compOrient(cv::Mat imuData){
	//cout << "Computing orientation from IMU" << endl;
	//cout << "imuData = " << imuData << endl;

	Mat ret(Mat::eye(4, 4, CV_32FC1));
	float yaw = imuData.at<float>(11)*PI/180;
	float pitch = imuData.at<float>(10)*PI/180;
	float roll = imuData.at<float>(9)*PI/180;
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
				0, cos(roll), sin(roll),
				0, -sin(roll), cos(roll));
	//cout << "Rx = " << Rx << endl;
	Mat tmp(Rx*Ry*Rz);
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

void MovementConstraints::updateCurPosCloudMapCenter(){
	if(robot->isImuOpen() && robot->isEncodersOpen()){
		std::chrono::high_resolution_clock::time_point imuTimestamp;
		Mat encodersCur = robot->getEncoderData();
		Mat imuCur = robot->getImuData(imuTimestamp);

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
		//cout << "compOrient(imuCur) = " << compOrient(imuCur) << endl;
		//cout << "Computing curPos" << endl;
		//cout << "encodersCur - encodersPrev = " << encodersCur - encodersPrev << endl;
		Mat trans = posMapCenterGlobal.inv()*compTrans(compOrient(imuPrev), encodersCur - encodersPrev);
		//cout << "trans = " << trans << endl;
		//cout << "Computing curTrans" << endl;
		Mat curTrans = Mat(curPosCloudMapCenter, Rect(3, 0, 1, 4)) + trans;
		//cout << "Computing curRot" << endl;

		Mat curRot = posMapCenterGlobal.inv()*compOrient(imuCur);
		curRot.copyTo(curPosCloudMapCenter);
		curTrans.copyTo(Mat(curPosCloudMapCenter, Rect(3, 0, 1, 4)));
		lck.unlock();
		//cout << "trans = " << trans << endl;
		//cout << "posMapCenterGlobal = " << posMapCenterGlobal << endl;
		cout << "curPosCloudMapCenter = " << curPosCloudMapCenter << endl;
		//cout << "curTrans = " << curTrans << endl;
		//cout << "curRot = " << curRot << endl;
		//cout << "imuPosGlobal.inv()*curPos*cameraOrigImu.inv() = " << endl << imuPosGlobal.inv()*curPos*cameraOrigImu.front().inv() << endl;
		//cout << "globalPos.inv()*curPos = " << globalPos.inv()*curPos << endl;

		imuCur.copyTo(imuPrev);
		encodersCur.copyTo(encodersPrev);
	}
}

void MovementConstraints::processPointCloud(){
	cout << "processPointCloud()" << endl;

	updateCurPosCloudMapCenter();

	if(hokuyo.isOpen()){
		std::chrono::high_resolution_clock::time_point hokuyoTimestamp;
		Mat hokuyoData = hokuyo.getData(hokuyoTimestamp);
		Mat hokuyoCurPoints(6, hokuyoData.cols, CV_32FC1);

		//cout << "Copying hokuyo data" << endl;
		//TODO remove invalid points
		Mat tmpX = -1*hokuyoData.rowRange(1, 2);
		//hokuyoCurPoints.rowRange(0, 1) = Scalar(-1)*hokuyoData.rowRange(1, 2);
		tmpX.copyTo(hokuyoCurPoints.rowRange(0, 1));
		hokuyoCurPoints.rowRange(1, 2) = Scalar(0);
		hokuyoData.rowRange(0, 1).copyTo(hokuyoCurPoints.rowRange(2, 3));
		hokuyoCurPoints.rowRange(3, 4) = Scalar(1);
		hokuyoData.rowRange(2, 4).copyTo(hokuyoCurPoints.rowRange(4, 6));

		//cout << "Moving to camera orig" << endl;
		hokuyoCurPoints.rowRange(0, 4) = cameraOrigLaser.inv()*hokuyoCurPoints.rowRange(0, 4);

		//cout << "Removing old points" << endl;
		static const int timeoutThres = 2000;
		//remove all points older than 2000 ms
		int pointsSkipped = 0;
		std::chrono::high_resolution_clock::time_point curTimestamp = std::chrono::high_resolution_clock::now();
		if(pointsQueue.size() > 0){
			//cout << "dt = " << std::chrono::duration_cast<std::chrono::milliseconds>(curTimestamp - pointsQueue.front().timestamp).count() << endl;
			while(std::chrono::duration_cast<std::chrono::milliseconds>(curTimestamp - pointsQueue.front().timestamp).count() > timeoutThres){
				pointsSkipped += pointsQueue.front().numPoints;
				pointsQueue.pop();
				if(pointsQueue.size() == 0){
					break;
				}
			}
		}

		//cout << "Moving pointCloudCameraMapCenter, pointsSkipped = " << pointsSkipped << endl;
		Mat tmpAllPoints(hokuyoCurPoints.rows, pointCloudCameraMapCenter.cols + hokuyoCurPoints.cols - pointsSkipped, CV_32FC1);
		if(!pointCloudCameraMapCenter.empty()){
			//cout << "copyTo" << endl;
			//TODO check copying when all points are being removed
			pointCloudCameraMapCenter.colRange(pointsSkipped,
												pointCloudCameraMapCenter.cols).
							copyTo(tmpAllPoints.colRange(0, pointCloudCameraMapCenter.cols - pointsSkipped));
		}
		//cout << "Addding hokuyoCurPoints" << endl;
		if(std::chrono::duration_cast<std::chrono::milliseconds>(curTimestamp - hokuyoTimestamp).count() <= timeoutThres){
			Mat curPointCloudCameraMapCenter(hokuyoCurPoints.rows, hokuyoCurPoints.cols, CV_32FC1);
			Mat tmpCurPoints = curPosCloudMapCenter*cameraOrigImu*hokuyoCurPoints.rowRange(0, 4);
			tmpCurPoints.copyTo(curPointCloudCameraMapCenter.rowRange(0, 4));
			hokuyoCurPoints.rowRange(4, 6).copyTo(curPointCloudCameraMapCenter.rowRange(4, 6));
			//cout << hokuyoCurPointsGlobal.channels() << ", " << hokuyoAllPointsGlobal.channels() << endl;
			curPointCloudCameraMapCenter.copyTo(tmpAllPoints.colRange(pointCloudCameraMapCenter.cols - pointsSkipped,
																		pointCloudCameraMapCenter.cols + hokuyoCurPoints.cols - pointsSkipped));

			pointsQueue.push(PointsPacket(hokuyoTimestamp, hokuyoCurPoints.cols));
		}
		std::unique_lock<std::mutex> lck(mtxPointCloud);
		pointCloudCameraMapCenter = tmpAllPoints;
		lck.unlock();
	}

	cout << "End processPointCloud()" << endl;
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
	cout << "getPointCloud()" << endl;
	Mat ret;
	std::unique_lock<std::mutex> lck(mtxPointCloud);
	pointCloudCameraMapCenter.copyTo(ret);
	if(!curPosCloudMapCenter.empty()){
		curPosMapCenter = curPosCloudMapCenter*cameraOrigImu;
	}
	lck.unlock();
	cout << "End getPointCloud()" << endl;
	return ret;
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
