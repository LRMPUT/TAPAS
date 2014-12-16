/*Copyright (c) 2014, TAPAS Team (cybair [at] put.poznan.pl), Poznan University of Technology
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
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
//STL
#include <cmath>
#include <sstream>
#include <algorithm>
//CUDA
#ifndef NO_CUDA
	#include <cuda_runtime.h>
#endif
//Robotour
#include "Camera.h"
#include "CameraCuda.h"
#include "../../Robot/Robot.h"

using namespace boost;
using namespace std;

/*#define CAMERA_Z 1
#define CAMERA_X_ANGLE 45
#define CAMERA_Y_ANGLE 45
#define CAMERAS_COUNT 2
#define ROWS 480
#define COLS 640*/

/*#define POLY_VERT 4
#define X_STEP 100
#define Y_STEP 100
#define X_RES 50
#define Y_RES 50
#define PLANE_Z -100*/
#define DRIVABLE_LABEL 1
#define LEFT_CAMERA 0
#define RIGHT_CAMERA 1

#define CHANNELS_USED 2
#define SAMPLE_PACK 1500

using namespace cv;
//using namespace gpu;
using namespace std;


Camera::Camera(MovementConstraints* imovementConstraints, TiXmlElement* settings) :
		movementConstraints(imovementConstraints),
		runThread(false)
{
	if(!settings){
		throw "Bad settings file - entry Camera not found";
	}

	readSettings(settings);

	cameras.resize(numCameras);
	classifiedImage.resize(numCameras);

#ifndef NO_CUDA
	int devCount;
	cudaGetDeviceCount(&devCount);
	cout << "Available CUDA devices: " <<  devCount << endl;
	cudaDeviceProp prop;
	cudaGetDeviceProperties(&prop, 0);
	cout << "Computing capability: " << prop.major << "." << prop.minor << endl;
	cout << "Max threads per block: " << prop.maxThreadsPerBlock << endl;
	cout << "Max grid dim: " << prop.maxGridSize[0] << "x" << prop.maxGridSize[1] << "x" << prop.maxGridSize[2] << endl;
	cout << "Number of processors: " << prop.multiProcessorCount << endl;
	cout << "Unified addressing: " << prop.unifiedAddressing << endl;
#endif //NO_CUDA

	if(learnEnabled){
		learnFromDir(learningDirs);
	}
	if(cacheLoadEnabled){
		readCache("cache/cameraCache");
	}
	cameraThread = std::thread(&Camera::run, this);
}

Camera::~Camera(){
	runThread = false;
	if(cameraThread.joinable()){
		cameraThread.join();
	}
	close();
	for(int i = 0; i < hierClassifiers.size(); i++){
		delete hierClassifiers[i];
	}
}

void Camera::computeConstraints(std::chrono::high_resolution_clock::time_point nextCurTimestamp){
//	cout << "Computing constraints" << endl;

	Mat votes(MAP_SIZE, MAP_SIZE, CV_32SC1, Scalar(0));
	Mat countVotes(MAP_SIZE, MAP_SIZE, CV_32SC1, Scalar(0));
	for(int cam = 0; cam < mapSegments.size(); cam++){
		if(!mapSegments[cam].empty() && !classifiedImage[cam].empty()){
			for(int r = 0; r < numRows; r++){
				for(int c = 0; c < numCols; c++){
					//cout << mapSegments[cam].at<int>(r, c) << endl;
					int x = mapSegments[cam].at<int>(r, c) / MAP_SIZE;
					int y = mapSegments[cam].at<int>(r, c) % MAP_SIZE;
					if(x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE){
						//cout << "(" << x << ", " << y << "), label " << classifiedImage[cam].at<int>(r, c) << endl;
						if(classifiedImage[cam].at<int>(r, c) >= 0){
							countVotes.at<int>(x, y)++;
							if(classifiedImage[cam].at<int>(r, c) != DRIVABLE_LABEL){
								votes.at<int>(x, y)++;
							}
						}
					}
				}
			}
		}
	}
	int nhood[][2] = {{-1, -1},
						{-1, 0},
						{-1, 1},
						{0, 1},
						{1, 1},
						{1, 0},
						{1, -1},
						{0, -1}};
	std::unique_lock<std::mutex> lck(mtxConstr);
	constraints = Mat(MAP_SIZE, MAP_SIZE, CV_32FC1, Scalar(0));
	for(int y = 0; y < MAP_SIZE; y++){
		for(int x = 0; x < MAP_SIZE; x++){
			if(countVotes.at<int>(x, y) > 0){
				constraints.at<float>(x, y) = (float)votes.at<int>(x, y)/countVotes.at<int>(x, y);
				//cout << x << ":" << y << " = " << (float)votes.at<int>(x, y)/countVotes.at<int>(x, y) << endl;
			}
			else{
				constraints.at<float>(x, y) = 0;
			}
		}
	}
	for(int y = 0; y < MAP_SIZE; y++){
		for(int x = 0; x < MAP_SIZE; x++){
		float maxNhVal = 0;
		for(int n = 0; n < sizeof(nhood)/sizeof(nhood[0]); n++){
			int nx = x + nhood[n][0];
			int ny = y + nhood[n][1];
			if(nx >= 0 && nx < MAP_SIZE && ny >= 0 && ny < MAP_SIZE){
				maxNhVal = max(maxNhVal, constraints.at<float>(nx, ny));
			}
		}
		constraints.at<float>(x, y) = min(constraints.at<float>(x, y), maxNhVal);
		//cout << x << ":" << y << " = " << (float)votes.at<int>(x, y)/countVotes.at<int>(x, y) << endl;
		}
	}
	curTimestamp = nextCurTimestamp;
	lck.unlock();
}

std::vector<cv::Mat> Camera::computeMapSegments(cv::Mat curPosImuMapCenter){
//	cout << "Computing map segments" << endl;
	//cout << "curPosMapCenter = " << curPosImuMapCenter << endl;
	//namedWindow("test");
	//mapSegments.clear();
	vector<Mat> ret;

	for(int cam = 0; cam < numCameras; cam++){
		ret.push_back(Mat(numRows, numCols, CV_32SC1, Scalar(-1)));

		//cout << "curPosImuMapCenter*cameraOrigImu[cam]" << endl;
		Mat curPosCameraMapCenterGlobal = imuOrigRobot*curPosImuMapCenter*cameraOrigImu[cam];
		Mat invCameraMatrix = cameraMatrix[cam].inv();

		for(int r = 0; r < numRows; r++){
			for(int c = 0; c < numCols; c++){
				Mat pointIm(3, 1, CV_32FC1);
				pointIm.at<float>(0) = c;
				pointIm.at<float>(1) = r;
				pointIm.at<float>(2) = 1;
				//cout << "invCameraMatrix*pointIm" << endl;
				Mat pointCamNN = invCameraMatrix*pointIm;
				float t31 = curPosCameraMapCenterGlobal.at<float>(2, 0);
				float t32 = curPosCameraMapCenterGlobal.at<float>(2, 1);
				float t33 = curPosCameraMapCenterGlobal.at<float>(2, 2);
				float t34 = curPosCameraMapCenterGlobal.at<float>(2, 3);
				//z coordinate == 0
				float s = (-t34) / (t31 * pointCamNN.at<float>(0) + t32 * pointCamNN.at<float>(1) + t33 * pointCamNN.at<float>(2)); //at z_glob = 0
				Mat pointCam(4, 1, CV_32FC1);
				pointCam.at<float>(0) = pointCamNN.at<float>(0) * s;
				pointCam.at<float>(1) = pointCamNN.at<float>(1) * s;
				pointCam.at<float>(2) = pointCamNN.at<float>(2) * s;
				pointCam.at<float>(3) = 1;
				//cout << "curPosCameraMapCenter*pointCam" << endl;
				Mat pointMapCenter = curPosImuMapCenter*cameraOrigImu[cam]*pointCam;
				//cout << "pointIm = " << pointIm << endl;
				//cout << "pointCamNN = " << pointCamNN << endl;
				//cout << "s = " << s << endl;
				//cout << "curPosCameraMapCenter = " << curPosCameraMapCenter << endl;
				//cout << "pointCam = " << pointCam << endl;
				//cout << "pointMapCenter = " << pointMapCenter << endl;
				//cout << "pointMapCenter.size() =" << pointMapCenter.size() << endl;
				//cout << "mapSegments[cam].size() = " << mapSegments[cam].size() << endl;
				int xSegm = pointMapCenter.at<float>(0)/MAP_RASTER_SIZE + MAP_SIZE/2;
				int ySegm = pointMapCenter.at<float>(1)/MAP_RASTER_SIZE + MAP_SIZE/2;
				//cout << r << ":" << c << " = (" << xSegm << ", " << ySegm << ")" << endl;
				ret[cam].at<int>(r, c) = xSegm*MAP_SIZE + ySegm;
				//cout << "ret[cam].at<int>(r, c) = " << ret[cam].at<int>(r, c) << endl;
				//cout << "End mapSegments[c].at<int>(r, c) =" << endl;
				//waitKey();
			}
		}
	}
//	cout << "End computing map segments" << endl;
	return ret;
}

#ifndef NO_CUDA
std::vector<cv::Mat> Camera::computeMapSegmentsGpu(cv::Mat curPosImuMapCenter){
//	cout << "Computing map segments" << endl;
	//namedWindow("test");
	//mapSegments.clear();
	vector<Mat> ret;

	for(int cam = 0; cam < numCameras; cam++){
		ret.push_back(Mat(numRows, numCols, CV_32SC1, Scalar(-1)));

		//cout << "curPosImuMapCenter*cameraOrigImu[cam]" << endl;
		Mat curPosCameraMapCenterGlobal = imuOrigRobot*curPosImuMapCenter*cameraOrigImu[cam];
		Mat curPosCameraMapCenterImu = curPosImuMapCenter*cameraOrigImu[cam];
		Mat invCameraMatrix = cameraMatrix[cam].inv();

		if(ret[cam].isContinuous() &&
				curPosCameraMapCenterGlobal.isContinuous() &&
				curPosCameraMapCenterImu.isContinuous() &&
				invCameraMatrix.isContinuous())
		{
			reprojectCameraPoints((float*)invCameraMatrix.data,
									(float*)NULL,
									(float*)curPosCameraMapCenterGlobal.data,
									(float*)curPosCameraMapCenterImu.data,
									numRows,
									numCols,
									(int*)ret[cam].data,
									MAP_SIZE,
									MAP_RASTER_SIZE);

		}
	}
	return ret;
}
#endif //NO_CUDA

std::vector<cv::Point2f> Camera::computePointProjection(const std::vector<cv::Point3f>& spPoints,
														int cameraInd)
{
	vector<Point2f> ret;
	projectPoints(	spPoints,
					Matx<float, 3, 1>(0, 0, 0),
					Matx<float, 3, 1>(0, 0, 0),
					cameraMatrix[cameraInd],
					distCoeffs[cameraInd],
					ret);
	return ret;
}

std::vector<cv::Point3f> Camera::computePointReprojection(	const std::vector<cv::Point2f>& imPoints,
															int cameraInd)
{

}

/*void Camera::addToLearnDatabase(cv::Mat samples, int label){
	Mat data[] = {samples};
	int channels[] = {0};
	int histSize[] = {bins};
	float range[] = {0, 256};
	const float* ranges[] = {range};
	Mat hist(1, bins, CV_32FC1);
	Entry newEntry;
	newEntry.label = label;
	newEntry.descriptor = Mat(1, CHANNELS_USED*bins, CV_32FC1);

	for(int i = 0; i < CHANNELS_USED; i++){
		channels[0] = i;
		calcHist(data, 1, channels, Mat(), hist, 1, histSize, ranges);
		hist.copyTo(newEntry.descriptor.colRange(bins*i, bins*(i + 1) - 1));
	}
	normalize(hist, hist, 1, 0, NORM_L1, -1);
	entries.push_back(newEntry);
}

void Camera::clearLearnDatabase(){
	entries.clear();
}

void Camera::learn(){
	if(entries.size() > 0){
		Mat allHist(entries.size(), entries[0].descriptor.cols, CV_32FC1);
		Mat allLabels(entries.size(), 1, CV_8UC1);
		for(int i = 0; i < entries.size(); i++){
			entries[i].descriptor.copyTo(allHist.rowRange(i, i));
			allLabels.at<unsigned char>(i) = entries[i].label;
		}
		svm.train(allHist, allLabels, Mat(), Mat(), svmParams);
	}
}*/

int Camera::selectPolygonPixels(std::vector<cv::Point2i> polygon, float regionId, cv::Mat& regionsOnImage){
	int polyCnt[] = {(int)polygon.size()};
	const Point2i* points[] = {polygon.data()};
	//Point2i array
	fillPoly(regionsOnImage, points, polyCnt, 1, Scalar(regionId));
	int count = regionsOnImage.rows * regionsOnImage.cols - countNonZero(regionsOnImage - Scalar(regionId));

	return count;
}

int Camera::selectPolygonPixels(std::vector<cv::Point2f> polygon, float regionId, cv::Mat& regionsOnImage){
	vector<Point2i> tmp;
	for(int v = 0; v < polygon.size(); v++){
		tmp.push_back(Point2i(cvRound(polygon[v].x), cvRound(polygon[v].y)));
	}
	return selectPolygonPixels(tmp, regionId, regionsOnImage);
}

/*cv::Mat Camera::compOrient(cv::Mat imuData){
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
				0, cos(roll), -sin(roll),
				0, sin(roll), cos(roll));
	//cout << "Rx = " << Rx << endl;
	Mat tmp(Rz*Ry*Rx);
	tmp.copyTo(ret(Rect(0, 0, 3, 3)));

	//cout << "End computing orientation from IMU" << endl;
	return ret;
}


cv::Mat Camera::compTrans(	cv::Mat orient,
							cv::Mat encodersDiff)
{
	static const float wheelCir = 178*PI;
	static const float wheelDistance = 432;
	static const int encodersCPR = 300;
	float sl = encodersDiff.at<float>(0)*wheelCir/encodersCPR;
	float sr = encodersDiff.at<float>(1)*wheelCir/encodersCPR;
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
	//cout << trans << endl << orient << endl;
	trans = orient*trans;
	return trans;
}*/

bool Camera::readLineFloat(std::ifstream& stream, Mat& data){
	data = Mat(0, 1, CV_32FC1);
	string line;
	getline(stream, line);
	//cout << "Got line: " << line << endl;
	if(stream.eof()){
		return false;
	}
	stringstream tmp(line);
	//double time;
	//tmp >> time;
	while(!tmp.eof()){
		float val;
		tmp >> val;
		if(tmp.fail()){
			break;
		}
		data.push_back(val);
	}
	data = data.reshape(0, 1);
	//cout << "returning matrix: " << data << endl;
	return true;
}

bool Camera::readLineInt(std::ifstream& stream, Mat& data){
	data = Mat(0, 1, CV_32SC1);
	string line;
	getline(stream, line);
	//cout << "Got line: " << line << endl;
	if(stream.eof()){
		return false;
	}
	stringstream tmp(line);
	//double time;
	//tmp >> time;
	while(!tmp.eof()){
		int val;
		tmp >> val;
		if(tmp.fail()){
			break;
		}
		data.push_back(val);
	}
	data = data.reshape(0, 1);
	//cout << "returning matrix: " << data << endl;
	return true;
}

void Camera::processDir(boost::filesystem::path dir,
						std::vector<cv::Mat>& images,
						std::vector<cv::Mat>& manualRegionsOnImages,
						std::vector<std::map<int, int> >& mapRegionIdToLabel,
						std::vector<cv::Mat>& terrains,
						std::vector<cv::Mat>& poses)
{
	cout << "processDir()" << endl;
	images.clear();
	manualRegionsOnImages.clear();
	mapRegionIdToLabel.clear();
	terrains.clear();
	poses.clear();

	//cout << dir.string() + "/hokuyo.data" << endl;
	ifstream hokuyoFile(dir.string() + "/hokuyo.data");
	if(!hokuyoFile.is_open()){
		cout << "Error - no hokuyo file" << endl;
		throw "Error - no hokuyo file";
	}
	ifstream imuFile(dir.string() + "/imu.data");
	if(!imuFile.is_open()){
		cout << "Error - no imu file" << endl;
		throw "Error - no imu file";
	}
	ifstream encodersFile(dir.string() + "/encoders.data");
	if(!encodersFile.is_open()){
		cout << "Error - no encoders file" << endl;
		throw "Error - no encoders file";
	}
	ifstream cameraFile(dir.string() + "/camera.data");
	if(!encodersFile.is_open()){
		cout << "Error - no camera file" << endl;
		throw "Error - no camera file";
	}
	Mat hokuyoAllPointsGlobal;
	Mat imuPrev, encodersPrev;
	Mat imuPosGlobal, curPos;

	std::queue<MovementConstraints::PointsPacket> pointsQueue;

	int hokuyoCurTime;
	hokuyoFile >> hokuyoCurTime;
	int imuCurTime;
	imuFile >> imuCurTime;
	int encodersCurTime;
	encodersFile >> encodersCurTime;
	namedWindow("test");

	/*static const int begInt = 1868;
	static const int endInt = 3333;
	map<int, Vec3b> colorMap;
	Mat intVal(1, endInt - begInt, CV_32SC1);
	for(int i = begInt; i < endInt; i++){
		intVal.at<int>(i - begInt) = i;
	}
	Mat colorVal;
	applyColorMap(intVal, colorVal, COLORMAP_JET);
	cout << "colorVal.type() = " << colorVal.type() << ", CV_32SC1 = " << CV_32SC1 << endl;
	for(int i = 0; i < intVal.cols; i++){
		colorMap[intVal.at<int>(i)] = colorVal.at<Vec3b>(i);
	}*/

	static Mat covarImage(3, 3, CV_32FC1, Scalar(0));
	static Mat meanImage(3, 1, CV_32FC1, Scalar(0));
	static int numPix = 0;

	static Mat covarLaser(2, 2, CV_32FC1, Scalar(0));
	static Mat meanLaser(2, 1, CV_32FC1, Scalar(0));
	static int numPts = 0;

	bool endFlag = false;
	while(!endFlag)
	{
		int cameraCurTime;
		cameraFile >> cameraCurTime;
		if(cameraFile.eof()){
			endFlag = true;
			break;
		}
		filesystem::path cameraImageFile;
		cameraFile >> cameraImageFile;
		cout << "Camera time: " << cameraCurTime << ", camera file: " << cameraImageFile.stem() << endl;

		while(hokuyoCurTime <= cameraCurTime && !hokuyoFile.eof()){
			cout << "Hokuyo time: " << hokuyoCurTime << endl;

			Mat hokuyoData, hokuyoCurPointsDist, hokuyoCurPointsInt;
			char tmpChar;
			hokuyoFile >> tmpChar;
			readLineInt(hokuyoFile, hokuyoCurPointsDist);
			hokuyoFile >> tmpChar;
			readLineInt(hokuyoFile, hokuyoCurPointsInt);
			//hokuyoCurPoints = Mat(6, hokuyoCurPointsDist.cols, CV_32FC1, Scalar(1));
			hokuyoData = Mat(0, 4, CV_32SC1);

			static const float stepAngle = 0.25*PI/180;
			float startAngle = -(hokuyoCurPointsDist.cols - 1)/2 * stepAngle;
			cout << "startAngle = " << startAngle << endl;
			for(int i = 0; i < hokuyoCurPointsDist.cols; i++){
				if(hokuyoCurPointsDist.at<int>(i) > 100){
					Mat curPoint(1, 4, CV_32FC1);
					curPoint.at<int>(0) = hokuyoCurPointsDist.at<int>(i)*cos(startAngle + i*stepAngle);
					curPoint.at<int>(1) = hokuyoCurPointsDist.at<int>(i)*sin(startAngle + i*stepAngle);
					curPoint.at<int>(2) = hokuyoCurPointsDist.at<int>(i);
					curPoint.at<int>(3) = hokuyoCurPointsInt.at<int>(i);
					hokuyoData.push_back(curPoint);
				}
			}
			cout << endl;
			hokuyoData = hokuyoData.t();

			//narrow hokuyo scan range 45 - 135
			//hokuyoCurPoints = hokuyoCurPoints.colRange(400, 681);

			if(imuPrev.empty()){
				readLineFloat(imuFile, imuPrev);
				imuFile >> imuCurTime;
			}
			if(encodersPrev.empty()){
				readLineInt(encodersFile, encodersPrev);
				encodersFile >> encodersCurTime;
			}
			if(!imuPrev.empty()){
				//cout << "imuPrev.size() = " << imuPrev.size() << endl;
				imuPrev = imuPrev.reshape(1, 3);
			}
			if(imuPosGlobal.empty()){
				imuPosGlobal = movementConstraints->compOrient(imuPrev);
				curPos = Mat::eye(4, 4, CV_32FC1);
			}
			Mat imuCur, encodersCur;

			while(imuCurTime <= hokuyoCurTime && !imuFile.eof()){
				//cout << "Imu time: " << imuCurTime << endl;
				readLineFloat(imuFile, imuCur);

				imuFile >> imuCurTime;
			}

			while(encodersCurTime <= hokuyoCurTime && !encodersFile.eof()){
				//cout << "Encoders time: " << encodersCurTime << endl;
				readLineInt(encodersFile, encodersCur);

				encodersFile >> encodersCurTime;
			}
			if(!imuCur.empty()){
				//cout << "imuCur.size() = " << imuCur.size() << endl;
				imuCur = imuCur.reshape(1, 3);
			}
			if(imuCur.empty()){
				imuPrev.copyTo(imuCur);
			}
			if(encodersCur.empty()){
				encodersPrev.copyTo(encodersCur);
			}
			//cout << "Computing curPos" << endl;
			//cout << "encodersCur = " << encodersCur << endl << "encodersPrev = " << encodersPrev << endl;
			curPos = movementConstraints->compNewPos(imuPrev, imuCur,
														encodersPrev, encodersCur,
														curPos, imuPosGlobal,
														movementConstraints->getPointCloudSettings());


			/*Mat trans = compTrans(compOrient(imuPrev), encodersCur - encodersPrev);
			cout << "trans = " << trans << endl;
			//cout << "Computing curTrans" << endl;
			Mat curTrans = Mat(curPos, Rect(3, 0, 1, 4)) + trans;
			//cout << "Computing curRot" << endl;

			Mat curRot = compOrient(imuCur)*cameraOrigImu.front();
			curRot.copyTo(curPos);
			curTrans.copyTo(Mat(curPos, Rect(3, 0, 1, 4)));*/
			//cout << "trans = " << trans << endl;
			//cout << "curTrans = " << curTrans << endl;
			//cout << "curRot = " << curRot << endl;
			cout << "curPos = " << endl << curPos << endl;
			//cout << "globalPos.inv()*curPos = " << globalPos.inv()*curPos << endl;

			std::chrono::duration<int,std::milli> durTmp(hokuyoCurTime);
			std::chrono::high_resolution_clock::time_point hokuyoTimestamp(durTmp);

			std::mutex mtxPointCloud;

			MovementConstraints::processPointCloud(hokuyoData,
													hokuyoAllPointsGlobal,
													pointsQueue,
													hokuyoTimestamp,
													hokuyoTimestamp,
													curPos,
													mtxPointCloud,
													cameraOrigLaser.front(),
													cameraOrigImu.front(),
													movementConstraints->getPointCloudSettings());

			//waitKey();
			/*Mat covarLaserCur, meanLaserCur;
			calcCovarMatrix(hokuyoCurPoints.rowRange(4, 6),
							covarLaserCur,
							meanLaserCur,
							CV_COVAR_NORMAL | CV_COVAR_SCALE | CV_COVAR_COLS,
							CV_32F);
			meanLaser = (meanLaser*numPts + meanLaserCur*hokuyoCurPoints.cols)/(numPts + hokuyoCurPoints.cols);
			covarLaser = (covarLaser*numPts + covarLaserCur*hokuyoCurPoints.cols)/(numPts + hokuyoCurPoints.cols);
			numPts += hokuyoCurPoints.cols;*/

			cout << "hokuyoAllPointsGlobal.cols = " << hokuyoAllPointsGlobal.cols << endl;

			imuCur.copyTo(imuPrev);
			encodersCur.copyTo(encodersPrev);
			hokuyoFile >> hokuyoCurTime;

			//curRot.copyTo(prevRot);
		}
		Mat terrain = hokuyoAllPointsGlobal.clone();
		terrain.rowRange(0, 4) = (curPos*cameraOrigImu.front()).inv()*hokuyoAllPointsGlobal.rowRange(0, 4);
		terrains.push_back(terrain);

		poses.push_back(curPos.clone());

		//cout << "Displaying test image from file: " << dir.string() + string("/") + cameraImageFile.string() << endl;
		Mat image = imread(dir.string() + string("/") + cameraImageFile.filename().string());
		//rectangle(image, Point(0, 0), Point(image.cols, 100), Scalar(0, 0, 0), -1);
		images.push_back(image.clone());
		if(image.data == NULL){
			throw "Bad image file";
		}
		Mat hokuyoAllPointsCamera = (curPos*cameraOrigImu.front()).inv()*hokuyoAllPointsGlobal.rowRange(0, 4);
		//cout << "Computing point projection" << endl;
		vector<Point2f> pointsImage;
		projectPoints(	hokuyoAllPointsCamera.rowRange(0, 3).t(),
						Matx<float, 3, 1>(0, 0, 0),
						Matx<float, 3, 1>(0, 0, 0),
						cameraMatrix.front(),
						distCoeffs.front(),
						pointsImage);
		//cout << "Drawing points" << endl;

		cout << "Number of points terrain = " << numPts << endl;
		cout << "mean terrain = " << meanLaser << endl;
		cout << "covar terrain = " << covarLaser << endl;

		Mat covarImageCur, meanImageCur;
		Mat imageHSV;
		cvtColor(image, imageHSV, CV_BGR2HSV);
		imageHSV = imageHSV.reshape(1, imageHSV.cols*imageHSV.rows);
		calcCovarMatrix(imageHSV,
						covarImageCur,
						meanImageCur,
						CV_COVAR_NORMAL | CV_COVAR_SCALE | CV_COVAR_ROWS,
						CV_32F);

		meanImage = (meanImage*numPix + meanImageCur.t()*imageHSV.rows)/(numPix + imageHSV.rows);
		covarImage = (covarImage*numPix + covarImageCur*imageHSV.rows)/(numPix + imageHSV.rows);
		numPix += imageHSV.rows;

		cout << "Number of points image = " << numPix << endl;
		cout << "mean image = " << meanImage << endl;
		cout << "covar image = " << covarImage << endl;

		//ofstream pointsFile("points/" + cameraImageFile.stem().string() + ".log");
		//if(!pointsFile.is_open()){
		//	cout << "pointsFile not opened" << endl;
		//	throw "pointsFile not opened";
		//}

		for(int p = 0; p < pointsImage.size(); p++){
			if(pointsImage[p].x >= 0 && pointsImage[p].x < image.cols &&
				pointsImage[p].y >= 0 && pointsImage[p].y < image.rows)
			{
				//pointsFile << pointsImage[p].x << " " << pointsImage[p].y << " " << hokuyoAllPointsGlobal.at<float>(5, p) << endl;
				//image.at<Vec3b>(pointsImage[p].y, pointsImage[p].x) = Vec3b(0, 0, 255);
				circle(image, Point(pointsImage[p].x, pointsImage[p].y), 2, Scalar(0, 0, 255), -1);
				//int intensity = hokuyoAllPointsGlobal.at<float>(5, p);
				//cout << intensity << endl;
				//if(colorMap.count(intensity) > 0){
				//	Vec3b color = colorMap[intensity];
					//cout << color << endl;
				//	circle(image, Point(pointsImage[p].x, pointsImage[p].y), 2, Scalar(color[0], color[1], color[2]), -1);
				//}
			}
		}
		imshow("test", image);
		waitKey(100);

		TiXmlDocument data(	dir.string() +
							string("/") +
							cameraImageFile.stem().string() +
							string(".xml"));
		if(!data.LoadFile()){
			cout << "Bad data file" << endl;
			throw "Bad data file";
		}
		TiXmlElement* pAnnotation = data.FirstChildElement("annotation");
		if(!pAnnotation){
			cout << "Bad data file - no annotation entry" << endl;
			throw "Bad data file - no annotation entry";
		}
		TiXmlElement* pFile = pAnnotation->FirstChildElement("filename");
		if(!pFile){
			cout << "Bad data file - no filename entry" << endl;
			throw "Bad data file - no filename entry";
		}

		Mat manualRegionsOnImageCur(image.rows, image.cols, CV_32SC1, Scalar(0));
		int manualRegionsCount = 0;
		map<int, int> mapRegionIdToLabelCur;

		TiXmlElement* pObject = pAnnotation->FirstChildElement("object");
		while(pObject){

			TiXmlElement* pPolygon = pObject->FirstChildElement("polygon");
			if(!pPolygon){
				cout << "Bad data file - no polygon inside object" << endl;
				throw "Bad data file - no polygon inside object";
			}
			vector<Point2i> poly;

			TiXmlElement* pPt = pPolygon->FirstChildElement("pt");
			while(pPt){
				int x = atoi(pPt->FirstChildElement("x")->GetText());
				int y = atoi(pPt->FirstChildElement("y")->GetText());
				poly.push_back(Point2i(x, y));
				pPt = pPt->NextSiblingElement("pt");
			}

			TiXmlElement* pAttributes = pObject->FirstChildElement("attributes");
			if(!pAttributes){
				cout << "Bad data file - no object attributes" << endl;
				throw "Bad data file - no object attributes";
			}
			string labelText = pAttributes->GetText();
			int label = -1;
			for(int i = 0; i < labels.size(); i++){
				if(labelText == labels[i]){
					label = i;
					break;
				}
			}
			if(label == -1){
				cout << "No such label found" << endl;
				throw "No such label found";
			}

			mapRegionIdToLabelCur[++manualRegionsCount] = label;
			//cout << "Selecting polygon pixels for label " << labels[label] <<  endl;
			selectPolygonPixels(poly, manualRegionsCount, manualRegionsOnImageCur);
			//cout << "End selecting" << endl;

			pObject = pObject->NextSiblingElement("object");
		}
		mapRegionIdToLabel.push_back(mapRegionIdToLabelCur);
		manualRegionsOnImages.push_back(manualRegionsOnImageCur);
	}
}

void Camera::learnFromDir(std::vector<boost::filesystem::path> dirs){
	cout << "Learning from dir" << endl;
	//namedWindow("segments");
	//namedWindow("original");
	std::vector<cv::Mat> images;
	std::vector<cv::Mat> manualRegionsOnImages;
	std::vector<std::map<int, int> > mapRegionIdToLabel;
	std::vector<cv::Mat> terrains;
	std::vector<cv::Mat> poses;

	for(int d = 0; d < dirs.size(); d++){
		std::vector<cv::Mat> tmpImages;
		std::vector<cv::Mat> tmpManualRegionsOnImages;
		std::vector<std::map<int, int> > tmpMapRegionIdToLabel;
		std::vector<cv::Mat> tmpTerrains;
		std::vector<cv::Mat> tmpPoses;
		processDir(	dirs[d],
					tmpImages,
					tmpManualRegionsOnImages,
					tmpMapRegionIdToLabel,
					tmpTerrains,
					tmpPoses);
		for(int i = 0; i < tmpImages.size(); i++){
			images.push_back(tmpImages[i]);
			manualRegionsOnImages.push_back(tmpManualRegionsOnImages[i]);
			mapRegionIdToLabel.push_back(tmpMapRegionIdToLabel[i]);
			terrains.push_back(tmpTerrains[i]);
			poses.push_back(tmpPoses[i]);
		}
	}

	cout << "images.size() = " << images.size() << endl << "manualRegionsOnImages.size() = " << manualRegionsOnImages.size() << endl;

	vector<Entry> dataset;

	for(int i = 0; i < images.size(); i++){
		cout << "Segmenting" << endl;
		//Mat autoRegionsOnImage = hierClassifiers.front()->segmentImage(images[i]);
#ifdef NO_CUDA
		vector<Mat> mapSegmentsOnImage = computeMapSegments(poses[i]);
#else
		vector<Mat> mapSegmentsOnImage = computeMapSegmentsGpu(poses[i]);
#endif
		Mat autoRegionsOnImage = mapSegmentsOnImage.front();
		cout << "Assigning manual ids" << endl;
		//rectangle(manualRegionsOnImages[i], Point(0, 0), Point(images[i].cols, 100), Scalar(0, 0, 0), -1);
		map<int, int> assignedManualId = hierClassifiers.front()->assignManualId(autoRegionsOnImage, manualRegionsOnImages[i]);
		cout << "Extracting entries" << endl;
		bool debug = false;
		//cout << "Looking for class " << 1 << endl;
		for(map<int, int>::iterator it = assignedManualId.begin(); it != assignedManualId.end(); it++){
			//cout << mapRegionIdToLabel[i][it->second] << endl;
			if(mapRegionIdToLabel[i][it->second] == 1){
				debug = true;
				break;
			}
		}
		//cout << "terrains[i].size() = " << terrains[i].size() << endl;
		vector<Entry> newData = hierClassifiers.front()->extractEntries(images[i],
																		terrains[i],
																		autoRegionsOnImage,
																		maskIgnore.front(),
																		entryWeightThreshold);

		for(int e = 0; e < newData.size(); e++){
			if(mapRegionIdToLabel[i].count(assignedManualId[newData[e].imageId]) > 0){
				newData[e].label = mapRegionIdToLabel[i][assignedManualId[newData[e].imageId]];
				dataset.push_back(newData[e]);
			}
		}


		vector<Scalar> colors;
		colors.push_back(Scalar(0, 255, 0));	//grass - green
		colors.push_back(Scalar(0, 0, 255));	//wood - red
		colors.push_back(Scalar(0, 255, 255));	//yellow - ceramic
		colors.push_back(Scalar(255, 0, 0));	//blue - asphalt

		Mat coloredOriginal = images[i].clone();

		for(int e = 0; e < newData.size(); e++){
			if(mapRegionIdToLabel[i].count(assignedManualId[newData[e].imageId]) > 0){
				int label = newData[e].label;
				if(label < 0 || label > 1){
					throw "Bad label";
				}
				coloredOriginal.setTo(colors[label], autoRegionsOnImage == newData[e].imageId);
			}
		}
		Mat visualization = coloredOriginal * 0.25 + images[i] * 0.75;
		//ret.setTo(Scalar(0, 0, 0), maskIgnore.front() != 0);

		imshow("original", visualization);
		imshow("segments", hierClassifiers.front()->colorSegments(autoRegionsOnImage));
		waitKey(100);
	}

	ofstream dataFile("data.log");
	dataFile.precision(15);
	for(int e = 0; e < dataset.size(); e++){
		dataFile << dataset[e].label << " ";
		for(int d = 0; d < dataset[e].descriptor.cols; d++){
			dataFile << d + 1 << ":" << dataset[e].descriptor.at<float>(d) << " ";
		}
		dataFile << dataset[e].descriptor.cols + 1 << ":-1" << endl;
	}
	dataFile.close();

	cout << "dataset.size() = " << dataset.size() << endl;

	map<int, double> sizeOfLabels;
	map<int, int> popOfLabels;
	for(int e = 0; e < dataset.size(); e++){
		sizeOfLabels[dataset[e].label] += dataset[e].weight;
		popOfLabels[dataset[e].label]++;
	}
	cout << "Before normalization" << endl;
	for(map<int, double>::iterator it = sizeOfLabels.begin(); it != sizeOfLabels.end(); ++it){
		cout << "label " << it->first << ", weight = " << it->second << endl;
	}
	for(map<int, int>::iterator it = popOfLabels.begin(); it != popOfLabels.end(); ++it){
		cout << "label " << it->first << ", pop = " << it->second << endl;
	}

	for(int e = 0; e < dataset.size(); e++){
		dataset[e].weight /= sizeOfLabels[dataset[e].label]*sizeOfLabels.size();
		//dataset[e].weight = (dataset[e].label == 1 ? 1 : 100);
	}

	sizeOfLabels.clear();
	for(int e = 0; e < dataset.size(); e++){
		sizeOfLabels[dataset[e].label] += dataset[e].weight;
	}
	cout << "After normalization" << endl;
	for(map<int, double>::iterator it = sizeOfLabels.begin(); it != sizeOfLabels.end(); ++it){
		cout << "label " << it->first << ", weight = " << it->second << endl;
	}

	cout << "crossValidate = " << crossValidate << endl;
	if(crossValidate){
		hierClassifiers.front()->crossValidateSVMs(dataset);
	}
	hierClassifiers.front()->train(dataset, labels.size());

	if(cacheSaveEnabled){
		saveCache("cache/cameraCache");
	}
}

void Camera::classifyFromDir(boost::filesystem::path dir){
	cout << "Classifying" << endl;
	for(int l = 0; l < labels.size(); l++){
		namedWindow(labels[l]);
	}
	namedWindow("segments");
	namedWindow("original");
	namedWindow("colored");

	vector<vector<int> > classResultsPix(labels.size(), vector<int>(labels.size(), 0));
	vector<vector<int> > classResultsSeg(labels.size(), vector<int>(labels.size(), 0));

	filesystem::directory_iterator endIt;
	for(filesystem::directory_iterator dirIt(dir); dirIt != endIt; dirIt++){
		if(dirIt->path().filename().string().find(".xml") != string::npos){
			TiXmlDocument data(dirIt->path().string());
			if(!data.LoadFile()){
				throw "Bad data file";
			}
			TiXmlElement* pAnnotation = data.FirstChildElement("annotation");
			if(!pAnnotation){
				throw "Bad data file - no annotation entry";
			}
			TiXmlElement* pFile = pAnnotation->FirstChildElement("filename");
			if(!pFile){
				throw "Bad data file - no filename entry";
			}
			Mat image = imread(dir.string() + string("/") + pFile->GetText());
			if(image.data == NULL){
				throw "Bad image file";
			}

			//loading map
			int imageNum;
			sscanf(pFile->GetText(), "camera%d.jpg", &imageNum);
			Mat terrain;
			char terrainFilename[200];
			sprintf(terrainFilename, "%smap%03d.log", (dir.string() + string("/")).c_str(), imageNum);
			ifstream terrainFile(terrainFilename);
			if(terrainFile.is_open() == false){
				throw "No map file";
			}
			double tmp;
			while(!terrainFile.eof()){
				Mat terrainPoint(1, 5, CV_32FC1);	//x, y, z, distance, intensity
				for(int i = 0; i < 5; i++){
					terrainFile >> tmp;
					terrainPoint.at<float>(0, i) = tmp;
				}
				terrain.push_back(terrainPoint);
			}
			terrain = terrain.t();

			Mat manualRegionsOnImage(image.rows, image.cols, CV_32SC1, Scalar(0));
			int manualRegionsCount = 0;
			map<int, int> mapRegionIdToLabel;

			TiXmlElement* pObject = pAnnotation->FirstChildElement("object");
			while(pObject){

				TiXmlElement* pPolygon = pObject->FirstChildElement("polygon");
				if(!pPolygon){
					throw "Bad data file - no polygon inside object";
				}
				vector<Point2i> poly;

				TiXmlElement* pPt = pPolygon->FirstChildElement("pt");
				while(pPt){
					int x = atoi(pPt->FirstChildElement("x")->GetText());
					int y = atoi(pPt->FirstChildElement("y")->GetText());
					poly.push_back(Point2i(x, y));
					pPt = pPt->NextSiblingElement("pt");
				}

				TiXmlElement* pAttributes = pObject->FirstChildElement("attributes");
				if(!pAttributes){
					throw "Bad data file - no object attributes";
				}
				string labelText = pAttributes->GetText();
				int label = 0;
				for(int i = 0; i < labels.size(); i++){
					if(labelText == labels[i]){
						label = i;
						break;
					}
				}

				mapRegionIdToLabel[++manualRegionsCount] = label;
				selectPolygonPixels(poly, manualRegionsCount, manualRegionsOnImage);

				pObject = pObject->NextSiblingElement("object");
			}
			Mat autoSegmented = hierClassifiers.front()->segmentImage(image, 200);
			map<int, int> assignedManualId = hierClassifiers.front()->assignManualId(autoSegmented, manualRegionsOnImage);
			imshow("original", image);
			imshow("segments", hierClassifiers.front()->colorSegments(autoSegmented));

			vector<vector<int> > curClassResultsPix(labels.size(), vector<int>(labels.size(), 0));
			vector<vector<int> > curClassResultsSeg(labels.size(), vector<int>(labels.size(), 0));

			vector<Mat> classificationResult = hierClassifiers.front()->classify(image, terrain, autoSegmented);
			for(int l = 0; l < labels.size(); l++){
				double minVal, maxVal;
				minMaxIdx(classificationResult[l], &minVal, &maxVal);
				cout << labels[l] << ", min = " << minVal << ", max = " << maxVal << endl;
				imshow(labels[l], classificationResult[l]);
			}
			vector<Scalar> colors;
			colors.push_back(Scalar(0, 255, 0));	//grass - green
			colors.push_back(Scalar(0, 0, 255));	//wood - red
			colors.push_back(Scalar(0, 255, 255));	//yellow - ceramic
			colors.push_back(Scalar(255, 0, 0));	//blue - asphalt
			Mat coloredOriginal(image.rows, image.cols, CV_8UC3);
			Mat bestLabels(image.rows, image.cols, CV_32SC1, Scalar(0));
			Mat bestScore(image.rows, image.cols, CV_32FC1, Scalar(-1));
			for(int l = 0; l < labels.size(); l++){
				Mat cmp;
				compare(bestScore, classificationResult[l], cmp, CMP_LE);
				bestLabels.setTo(l, cmp);
				bestScore = max(bestScore, classificationResult[l]);
			}
			for(int l = 0; l < labels.size(); l++){
				coloredOriginal.setTo(colors[l], bestLabels == l);
			}
			imshow("colored", coloredOriginal * 0.25 + image * 0.75);

			set<int> counted;
			for(int r = 0; r < image.rows; r++){
				for(int c = 0; c < image.cols; c++){
					int pred = bestLabels.at<int>(r, c);
					if(mapRegionIdToLabel.count(assignedManualId[autoSegmented.at<int>(r, c)]) > 0){
						if(counted.count(autoSegmented.at<int>(r, c)) == 0){
							int groundTrue = mapRegionIdToLabel[assignedManualId[autoSegmented.at<int>(r, c)]];
							curClassResultsSeg[groundTrue][pred]++;
							counted.insert(autoSegmented.at<int>(r, c));
						}
					}
					if(mapRegionIdToLabel.count(manualRegionsOnImage.at<int>(r, c)) > 0){
						int groundTrue = mapRegionIdToLabel[manualRegionsOnImage.at<int>(r, c)];
						curClassResultsPix[groundTrue][pred]++;
					}
				}
			}

			for(int t = 0; t < labels.size(); t++){
				for(int p = 0; p < labels.size(); p++){
					classResultsPix[t][p] += curClassResultsPix[t][p];
					classResultsSeg[t][p] += curClassResultsSeg[t][p];
				}
			}

			cout << "Current frame pixel results: " << endl;
			for(int t = 0; t < labels.size(); t++){
				cout << "true = " << t << ": ";
				for(int p = 0; p < labels.size(); p++){
					cout << curClassResultsPix[t][p] << ", ";
				}
				cout << endl;
			}

			cout << "Current frame segment results: " << endl;
			for(int t = 0; t < labels.size(); t++){
				cout << "true = " << t << ": ";
				for(int p = 0; p < labels.size(); p++){
					cout << curClassResultsSeg[t][p] << ", ";
				}
				cout << endl;
			}

			waitKey();
		}
	}

	cout << "General pixel results: " << endl;
	for(int t = 0; t < labels.size(); t++){
		cout << "true = " << t << ": ";
		for(int p = 0; p < labels.size(); p++){
			cout << classResultsPix[t][p] << ", ";
		}
		cout << endl;
	}

	cout << "General segment results: " << endl;
	for(int t = 0; t < labels.size(); t++){
		cout << "true = " << t << ": ";
		for(int p = 0; p < labels.size(); p++){
			cout << classResultsSeg[t][p] << ", ";
		}
		cout << endl;
	}

	cout << "End classifying" << endl;
}

/*cv::Mat Camera::classifySlidingWindow(cv::Mat image){
	//wxDateTime StartTime = wxDateTime::UNow();

	const int rows = image.rows;
	const int cols = image.cols;
	const int step = classifyGrid;

	GpuMat imageHSV(rows, cols, CV_8UC3);
	GpuMat imageH(rows, cols, CV_8UC1);
	GpuMat imageS(rows, cols, CV_8UC1);
	GpuMat imageV(rows, cols, CV_8UC1);
	GpuMat out[] = {imageH, imageS, imageV};

	imageHSV.upload(image);
	cvtColor(imageHSV, imageHSV, CV_BGR2HSV);
	split(imageHSV, out);

	//wxDateTime UploadTime = wxDateTime::UNow();

	vector<Mat> votes(labels.size());

	for (int i = 0; i < (int)votes.size(); i++)
	{
		// Sepatate Mat for each entry
		votes[i] = Mat(image.rows, image.cols, CV_8U, Scalar(0));
	}

	GpuMat*** entries = new GpuMat**[rows/step];
	for(int row = 0; row < rows/step; row++){
		entries[row] = new GpuMat*[cols/step];
	}
	for(int row = 0; row < rows/step; row++){
		for(int col = 0; col < cols/step; col++){
			entries[row][col] = new GpuMat(1, 2*bins, CV_32SC1);
		}
	}

	cout << "Calculating histograms" << endl;
	GpuMat buf(1, bins, CV_32SC1);
	for (int row = step; row <= rows; row += step)
	{
		for (int col = step; col <= cols; col += step)
		{
			const int MinC = col - step;
		    const int MaxC = col;
		    const int MinR = row - step;
		    const int MaxR = row;

		    //cout << "MinX: " << MinX << " MinY: " << MinY << " MaxX " << MaxX << " MaxY " << MaxY << "\n";
		    GpuMat RoiH = GpuMat(imageH, Rect(Point(MinC, MinR), Point(MaxC, MaxR)));
		    GpuMat RoiS = GpuMat(imageS, Rect(Point(MinC, MinR), Point(MaxC, MaxR)));

		    //cout << "Calculating hist for row = " << row << ", col = " << col << endl;
			GenerateColorHistHSVGpu(RoiH, RoiS, *entries[(row - 1)/step][(col - 1)/step], buf);

		}
	}

	//wxDateTime HistTime = wxDateTime::UNow();

	cout << "Classifing" << endl;

    Mat histSum(1, 2*bins, CV_32FC1);
    GpuMat histSumGpu(1, 2*bins, CV_32FC1);
    buf = GpuMat(1, 2*bins, CV_32FC1);
	for (int row = classifyGrid; row <= rows; row += step)
	{
		for (int col = classifyGrid; col <= cols; col += step)
		{
			const int MinC = col - classifyGrid;
		    const int MaxC = col;
		    const int MinR = row - classifyGrid;
		    const int MaxR = row;

		    int idxR = (row - 1)/step;
		    int idxC = (col - 1)/step;
		    int subGrids = classifyGrid/step;
		    histSumGpu = Scalar(0);
		    for(int subRow = idxR - subGrids + 1; subRow <= idxR; subRow++){
			    for(int subCol = idxC - subGrids + 1; subCol <= idxC; subCol++){
			    	add(histSumGpu, *entries[subRow][subCol], histSumGpu);
			    }
		    }
		    normalize(histSum, histSum, 1, 0, NORM_L1, -1, buf);
		    histSumGpu.download(histSum);
		    unsigned int predictedLabel = 0;
		    predictedLabel = svm.predict(histSum);
		    //cout << WordPredictedLabel << endl;
		    //EndTimeClass = wxDateTime::UNow();

		    Mat Mask = Mat(rows, cols, CV_8U, Scalar(0));
		    rectangle(Mask, Point(MinC, MinR), Point(MaxC, MaxR), Scalar(0x1), CV_FILLED);
		    votes[predictedLabel] +=  Mask;
		}
	}

	for(int row = 0; row < rows/step; row++){
		for(int col = 0; col < cols/step; col++){
			delete entries[row][col];
		}
	}
	for(int row = 0; row < rows/step; row++){
		delete[] entries[row];
	}
	delete[] entries;

	//wxDateTime ClassTime = wxDateTime::UNow();

	//cout << "Uploading and converting time: " << (UploadTime - StartTime).Format(wxString::FromAscii("%M:%S:%l")).ToAscii().data() << "\n";
	//cout << "Calculating histograms time: " << (HistTime - UploadTime).Format(wxString::FromAscii("%M:%S:%l")).ToAscii().data() << "\n";
	//cout << "Classifing time: " << (ClassTime - HistTime).Format(wxString::FromAscii("%M:%S:%l")).ToAscii().data() << "\n";

}

void Camera::GenerateColorHistHSVGpu(
		const cv::gpu::GpuMat& ImageH,
		const cv::gpu::GpuMat& ImageS,
		cv::gpu::GpuMat& result,
		cv::gpu::GpuMat& buf)
{
	GpuMat HistH(1, bins, CV_32SC1);
	GpuMat HistS(1, bins, CV_32SC1);

	if(bins != 256){
		throw "Number of bins must be equal to 256";
	}

	calcHist(ImageH, HistH, buf);
	calcHist(ImageS, HistS, buf);

	GpuMat partH = result.colRange(0, bins);
	GpuMat partS = result.colRange(bins, 2*bins);

	HistH.copyTo(partH);
	HistS.copyTo(partS);

	result.convertTo(result, CV_32F);
}*/

//Run as separate thread
void Camera::run(){

	try{
		while(runThread){
			if(debugLevel >= 1){
				cout << "Camera run" << endl;
			}
			std::chrono::high_resolution_clock::time_point timeBegin;
			std::chrono::high_resolution_clock::time_point timeEndMapSegments;
			std::chrono::high_resolution_clock::time_point timeEndClassification;
			std::chrono::high_resolution_clock::time_point timeEndConstraints;
			Mat curPosImuMapCenter;
			std::chrono::high_resolution_clock::time_point nextCurTimestamp = std::chrono::high_resolution_clock::now();
			Mat pointCloudImu = movementConstraints->getPointCloud(curPosImuMapCenter);
			if(!curPosImuMapCenter.empty()){
				timeBegin = std::chrono::high_resolution_clock::now();
				vector<Mat> cameraData = this->getData();
				if(debugLevel >= 1){
					cout << "Computing map segments" << endl;
				}
	#ifdef NO_CUDA
				mapSegments = computeMapSegments(curPosImuMapCenter);
	#else
				mapSegments = computeMapSegmentsGpu(curPosImuMapCenter);
	#endif
				timeEndMapSegments = std::chrono::high_resolution_clock::now();
				for(int c = 0; c < numCameras; c++){
					if(cameras[c].isOpened()){
						if(!cameraData[c].empty()){
							Mat pointCloudCamera;
							if(!pointCloudImu.empty()){
								Mat pointCloudCamera(pointCloudImu.rows, pointCloudImu.cols, CV_32FC1);
								pointCloudImu.rowRange(4, 6).copyTo(pointCloudCamera.rowRange(4, 6));
								Mat tmpPointCoords = cameraOrigImu[c] * pointCloudImu.rowRange(0, 4);
								tmpPointCoords.copyTo(pointCloudCamera.rowRange(0, 4));
							}
							if(debugLevel >= 1){
								cout << "Classification" << endl;
							}
							vector<Mat> classRes = hierClassifiers[c]->classify(cameraData[c],
																				pointCloudCamera,
																				mapSegments[c],
																				maskIgnore[c],
																				entryWeightThreshold);
							timeEndClassification = std::chrono::high_resolution_clock::now();
							//cout << "End classification" << endl;
							Mat bestLabels(numRows, numCols, CV_32SC1, Scalar(-1));
							Mat bestScore(numRows, numCols, CV_32FC1, Scalar(-1));
							for(int l = 0; l < labels.size(); l++){
								Mat cmp;
								//cout << "classRes[l].size() = " << classRes[l].size() << endl;
								//cout << "bestScore.rows = " << bestScore.rows << endl;
								compare(bestScore, classRes[l], cmp, CMP_LE);
								//cout << "bestLabels.size() = " << bestLabels.size() << endl;
								bestLabels.setTo(l, cmp);
								//cout << "max" << endl;
								bestScore = max(bestScore, classRes[l]);
							}
							classifiedImage[c] = bestLabels;
							//classifiedImage[c] = Mat(numRows, numCols, CV_32SC1, Scalar(0));
							if(debugLevel >= 1){
								cout << "Copying classified image" << endl;
							}
							std::unique_lock<std::mutex> lck(mtxClassIm);
							classifiedImage[c].copyTo(sharedClassifiedImage);
							cameraData[c].copyTo(sharedOriginalImage);
							lck.unlock();
						}
					}
				}
				computeConstraints(nextCurTimestamp);
				timeEndConstraints = std::chrono::high_resolution_clock::now();
			}
			if(debugLevel >= 1){
				cout << "Map segments time: " << std::chrono::duration_cast<std::chrono::milliseconds>(timeEndMapSegments - timeBegin).count() << " ms" << endl;
				cout << "Classification time: " << std::chrono::duration_cast<std::chrono::milliseconds>(timeEndClassification - timeEndMapSegments).count() << " ms" << endl;
				cout << "Constraints time: " << std::chrono::duration_cast<std::chrono::milliseconds>(timeEndConstraints - timeEndClassification).count() << " ms" << endl;
			}

			std::chrono::milliseconds duration(100);
			std::this_thread::sleep_for(duration);
		}
	}
	catch(char const* error){
		cout << error << endl;
	}
	catch(...){
		cout << "Camera unrecognized exception" << endl;
		exit(-1);
	}
}



void Camera::readSettings(TiXmlElement* settings){
	if(settings->QueryBoolAttribute("runThread", &runThread) != TIXML_SUCCESS){
		throw "Bad settings file - no runThread setting for Camera";
	}

	if(settings->QueryIntAttribute("number", &numCameras) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of cameras";
	}
	if(settings->QueryIntAttribute("rows", &numRows) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of rows";
	}
	if(settings->QueryIntAttribute("cols", &numCols) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of cols";
	}
	if(settings->QueryIntAttribute("entryWeightThreshold", &entryWeightThreshold) != TIXML_SUCCESS){
		throw "Bad settings file - wrong entryWeightThreshold";
	}
	if(settings->QueryIntAttribute("debug", &debugLevel) != TIXML_SUCCESS){
		throw "Bad settings file - wrong debug level";
	}


	cacheSaveEnabled = true;
	TiXmlElement* pPtr = settings->FirstChildElement("cache");
	if(!pPtr){
		throw "Bad settings file - no cache setting for Camera";
	}
	if(pPtr->QueryBoolAttribute("saveEnabled", &cacheSaveEnabled) != TIXML_SUCCESS){
		cout << "Warning - no cacheSaveEnabled setting for Camera";
	}

	cacheLoadEnabled = true;
	if(pPtr->QueryBoolAttribute("loadEnabled", &cacheLoadEnabled) != TIXML_SUCCESS){
		cout << "Warning - no cacheLoadEnabled setting for Camera";
	}

	crossValidate = false;

	pPtr = settings->FirstChildElement("learning");
	if(!pPtr){
		throw "Bad settings file - no learning settings";
	}
	if(pPtr->QueryBoolAttribute("enabled", &learnEnabled) != TIXML_SUCCESS){
		cout << "Warning - no learnEnabled settings for Camera";
	}

	const char* learnText = pPtr->GetText();
	char dirText[100];
	//cout << "pPtr->GetText: " << pPtr->GetText() << endl;
	cout << "Learning dirs: " << endl;
	int lenText = strlen(learnText);
	const char* pos = find(learnText, learnText + lenText, '"');
	while(pos < learnText + lenText){
		const char* dirTextEnd = find(pos + 1, learnText + lenText, '"');
		if(dirTextEnd == learnText + lenText){
			break;
		}
		cout << string(pos + 1, dirTextEnd - pos - 1) << endl;
		learningDirs.push_back(string(pos + 1, dirTextEnd - pos - 1));
		pos = find(dirTextEnd + 1, learnText + lenText, '"');
	}

	pPtr = settings->FirstChildElement("labels");
	if(!pPtr){
		throw "Bad settings file - no labels settings";
	}
	TiXmlElement* pLabel = pPtr->FirstChildElement("label");
	while(pLabel){
		string text;
		int id;
		pLabel->QueryStringAttribute("text", &text);
		pLabel->QueryIntAttribute("id", &id);
		if(labels.size() <= id){
			labels.resize(id + 1);
		}
		labels[id] = text;
		pLabel = pLabel->NextSiblingElement("label");
	}

	imuOrigRobot = readMatrixSettings(settings, "imu_position_robot", 4, 4);

	pPtr = settings->FirstChildElement("sensor");
	cameraOrigImu.resize(numCameras);
	cameraOrigLaser.resize(numCameras);
	cameraMatrix.resize(numCameras);
	distCoeffs.resize(numCameras);
	hierClassifiers.resize(numCameras);
	maskIgnore.resize(numCameras);
	for(int i = 0; i < numCameras; i++){
		if(!pPtr){
			throw "Bad settings file - no sensor settings";
		}
		string tmp;
		pPtr->QueryStringAttribute("id", &tmp);
		int idx = 0;
		if(tmp == "left"){
			idx = LEFT_CAMERA;
		}
		else if(tmp == "right"){
			idx = RIGHT_CAMERA;
		}
		else{
			throw "Bad settings file - wrong camera id";
		}

		cameraOrigImu[idx] = readMatrixSettings(pPtr, "imu_position_camera", 4, 4).t();
		cameraOrigLaser[idx] = readMatrixSettings(pPtr, "position_laser", 4, 4);
		cameraMatrix[idx] = readMatrixSettings(pPtr, "camera_matrix", 3, 3);
		distCoeffs[idx] = readMatrixSettings(pPtr, "dist_coeffs", 1, 5);
		maskIgnore[idx] = Mat(numRows, numCols, CV_32SC1, Scalar(0));

		TiXmlElement* pMaskIgnore = pPtr->FirstChildElement("mask_ignore");
		if(!pMaskIgnore){
			throw "Bad settings file - no mask_ignore";
		}
		TiXmlElement* pPolygon = pMaskIgnore->FirstChildElement("polygon");
		if(!pPolygon){
			cout << "Bad settings file - no polygon inside mask_ignore" << endl;
			throw "Bad settings file - no polygon inside mask_ignore";
		}
		while(pPolygon){
			vector<Point2i> poly;

			TiXmlElement* pPt = pPolygon->FirstChildElement("pt");
			while(pPt){
				int x = atoi(pPt->FirstChildElement("x")->GetText());
				int y = atoi(pPt->FirstChildElement("y")->GetText());
				poly.push_back(Point2i(x, y));
				pPt = pPt->NextSiblingElement("pt");
			}
			selectPolygonPixels(poly, 1, maskIgnore[idx]);
			pPolygon = pPolygon->NextSiblingElement("polygon");
		}

		hierClassifiers[idx] = new HierClassifier(cameraMatrix[idx]);

		pPtr = pPtr->NextSiblingElement("sensor");
	}

	pPtr = settings->FirstChildElement("HierClassifier");
	if(!pPtr){
		throw "Bad settings file - no HierClassifier settings";
	}
	for(int i = 0; i < hierClassifiers.size(); i++){
		hierClassifiers[i]->loadSettings(pPtr);
	}

	/*svmParams = CvSVMParams();	//default values
	svmParams.kernel_type = kernelType;
	svmParams.svm_type = svmType;
	svmParams.degree = degree;
	svmParams.gamma = gamma;*/
}

cv::Mat Camera::readMatrixSettings(TiXmlElement* parent, const char* node, int rows, int cols){
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

void Camera::readCache(boost::filesystem::path cacheFile){
	/*entries.clear();
	TiXmlDocument doc(cacheFile.c_str());
	if(!doc.LoadFile()){
		throw "Could not load cache file";
	}
	TiXmlElement* pDatabase = doc.FirstChildElement("database");
	if(!pDatabase){
		throw "Bad cache file - database not found";
	}
	TiXmlElement* pEntry = pDatabase->FirstChildElement("entry");
	while(pEntry){
		Entry entry;
		int descLength;
		pEntry->QueryIntAttribute("desc_length", &descLength);
		entry.descriptor = Mat(descLength, 1, CV_32FC1);
		pEntry->QueryIntAttribute("label", &entry.label);
		stringstream tmpStr(pEntry->GetText());
		for(int i = 0; i < descLength; i++){
			float tmp;
			tmpStr >> tmp;
			entry.descriptor.at<float>(i) = tmp;
		}
		entries.push_back(entry);
		pEntry = pEntry->NextSiblingElement("entry");
	}*/
	for(int c = 0; c < numCameras; c++){
		char buffer[10];
		sprintf(buffer, "%02d.xml", c);
		hierClassifiers[c]->loadCache(cacheFile.string() + buffer);
	}
}

void Camera::saveCache(boost::filesystem::path cacheFile){
	/*TiXmlDocument doc;
	TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "UTF-8", "");
	doc.LinkEndChild(decl);
	TiXmlElement* pDatabase = new TiXmlElement("database");
	doc.LinkEndChild(pDatabase);
	for(int entr = 0; entr < entries.size(); entr++){
		TiXmlElement* pEntry = new TiXmlElement("entry");
		pDatabase->LinkEndChild(pEntry);
		pEntry->SetAttribute("label", entries[entr].label);
		stringstream tmpStr;
		for(int i = 0; i < entries[entr].descriptor.cols; i++){
			tmpStr << entries[entr].descriptor.at<float>(i) << " ";
		}
		//TODO correct using TiXmlText
		pEntry->SetValue(tmpStr.str());
	}
	doc.SaveFile(cacheFile.c_str());*/
	for(int c = 0; c < numCameras; c++){
		char buffer[10];
		sprintf(buffer, "%02d.xml", c);
		hierClassifiers[c]->saveCache(cacheFile.string() + buffer);
	}
}

//Inserts computed constraints into map
void Camera::insertConstraints(cv::Mat map,
								std::chrono::high_resolution_clock::time_point curTimestampMap)
{
	std::unique_lock<std::mutex> lck(mtxConstr);
	if(!constraints.empty() && curTimestampMap < curTimestamp){
		for(int x = 0; x < MAP_SIZE; x++){
			for(int y = 0; y < MAP_SIZE; y++){
				map.at<float>(x, y) = max(map.at<float>(x, y), constraints.at<float>(x, y));
				//cout << "constraints.at<float>(x, y) = " << constraints.at<float>(x, y) << endl;
				//cout << "map.at<float>(x, y) = " << map.at<float>(x, y) << endl;
			}
		}
	}
	lck.unlock();
}

//CV_8UC3 2x640x480: left, right image
const std::vector<cv::Mat> Camera::getData(){
	//empty matrix
	std::unique_lock<std::mutex> lck(mtxDevice);
	vector<Mat> ret;
	for(int i = 0; i < cameras.size(); i++){
		cameras[i].grab();
	}
	for(int i = 0; i < cameras.size(); i++){
		Mat tmp;
		cameras[i].retrieve(tmp);
		//cout << "tmp.cols: " << tmp.cols << ", tmp.rows: " << tmp.rows << endl;
		if(!tmp.empty() && (tmp.rows != numRows || tmp.cols != numCols)){
			Mat tmpResized;
			resize(tmp, tmpResized, Size(numCols, numRows));
			tmp = tmpResized;
		}
		ret.push_back(tmp);
	}
	lck.unlock();
	return ret;
}

cv::Mat Camera::getClassifiedImage(){
	vector<Scalar> colors;
	colors.push_back(Scalar(0, 255, 0));	//grass - green
	colors.push_back(Scalar(0, 0, 255));	//wood - red
	colors.push_back(Scalar(0, 255, 255));	//yellow - ceramic
	colors.push_back(Scalar(255, 0, 0));	//blue - asphalt

	std::unique_lock<std::mutex> lck(mtxClassIm);
	Mat ret;

	if(!sharedClassifiedImage.empty() && !sharedOriginalImage.empty()){
		Mat coloredOriginal = sharedOriginalImage.clone();
		for(int l = 0; l < labels.size(); l++){
			coloredOriginal.setTo(colors[l], sharedClassifiedImage == l);
		}
		ret = coloredOriginal * 0.25 + sharedOriginalImage * 0.75;
		ret.setTo(Scalar(0, 0, 0), maskIgnore.front() != 0);
	}
	lck.unlock();

	return ret;
}

void Camera::open(std::vector<std::string> device){
	for(int i = 0; i < min((int)device.size(), numCameras); i++){
		cout << "Opening device: " << device[i] << endl;
		cameras[i].open(0);
		if(!cameras[i].isOpened()){
			throw "Cannot open camera device";
		}
	}
}

void Camera::close(){
	cout << "Closing cameras" << endl;
	for(int i = 0; i < cameras.size(); i++){
		cameras[i].release();
	}
	cout << "End closing cameras" << endl;
}

bool Camera::isOpen(){
	return cameras.front().isOpened();
}
