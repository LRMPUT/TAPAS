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

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/viz/viz3d.hpp>
#include <opencv2/viz/vizcore.hpp>
//STL
#include <cmath>
#include <sstream>
#include <algorithm>
#include <thread>
//CUDA
#ifndef NO_CUDA
	#include <cuda_runtime.h>
#endif
//TAPAS
#include "Camera.h"
#include "CameraCuda.h"
#include "../../Robot/Robot.h"
#include "Pgm/CustFeature.h"
#include "Pgm/Inference.h"
#include "Pgm/ParamEst.h"
#include "../../Planning/LocalPlanner.h"

using namespace boost;
using namespace std;

#define DRIVABLE_LABEL 1
#define LEFT_CAMERA 0
#define RIGHT_CAMERA 1

//#define CHANNELS_USED 2
//#define SAMPLE_PACK 1500

using namespace cv;
//using namespace gpu;
using namespace std;

#ifndef NO_CUDA
// This will output the proper CUDA error strings in the event that a CUDA host call returns an error
#define checkCudaErrors(err)  __checkCudaErrors (err, __FILE__, __LINE__)

inline void __checkCudaErrors(cudaError err, const char *file, const int line )
{
    if(cudaSuccess != err)
    {
        fprintf(stderr, "%s(%i) : CUDA Runtime API error %d: %s.\n",file, line, (int)err, cudaGetErrorString( err ) );
        exit(-1);
    }
}
#endif

Camera::Camera(MovementConstraints* imovementConstraints, TiXmlElement* settings) :
		movementConstraints(imovementConstraints),
		runThread(false)
{
	if(!settings){
		throw "Bad settings file - entry Camera not found";
	}

	readSettings(settings);

	cameras.resize(cameraParams.numCameras);
	classifiedImage.resize(cameraParams.numCameras);

#ifndef NO_CUDA
	int devCount;
	cout << "Making device query" << endl;
	checkCudaErrors(cudaGetDeviceCount(&devCount));
	cout << "Available CUDA devices: " <<  devCount << endl;
	cudaDeviceProp prop;
	checkCudaErrors(cudaGetDeviceProperties(&prop, 0));
	cout << "Computing capability: " << prop.major << "." << prop.minor << endl;
	cout << "Max threads per block: " << prop.maxThreadsPerBlock << endl;
	cout << "Max grid dim: " << prop.maxGridSize[0] << "x" << prop.maxGridSize[1] << "x" << prop.maxGridSize[2] << endl;
	cout << "Number of processors: " << prop.multiProcessorCount << endl;
	cout << "Unified addressing: " << prop.unifiedAddressing << endl;
#endif //NO_CUDA

	if(cameraParams.learnEnabled){
		learnFromDir(cameraParams.learningDirs);
	}
	if(cameraParams.cacheLoadEnabled){
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

//void Camera::computeConstraints(std::vector<cv::Mat> mapSegments,
//								std::chrono::high_resolution_clock::time_point nextCurTimestamp)
//{
////	cout << "Computing constraints" << endl;
//
//	Mat votes(MAP_SIZE, MAP_SIZE, CV_32SC1, Scalar(0));
//	Mat countVotes(MAP_SIZE, MAP_SIZE, CV_32SC1, Scalar(0));
//	for(int cam = 0; cam < mapSegments.size(); cam++){
//		if(!mapSegments[cam].empty() && !classifiedImage[cam].empty()){
//			for(int r = 0; r < numRows; r++){
//				for(int c = 0; c < numCols; c++){
//					//cout << mapSegments[cam].at<int>(r, c) << endl;
//					int x = mapSegments[cam].at<int>(r, c) / MAP_SIZE;
//					int y = mapSegments[cam].at<int>(r, c) % MAP_SIZE;
//					if(x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE){
//						//cout << "(" << x << ", " << y << "), label " << classifiedImage[cam].at<int>(r, c) << endl;
//						if(classifiedImage[cam].at<int>(r, c) >= 0){
//							countVotes.at<int>(x, y)++;
//							if(classifiedImage[cam].at<int>(r, c) != DRIVABLE_LABEL){
//								votes.at<int>(x, y)++;
//							}
//						}
//					}
//				}
//			}
//		}
//	}
//	int nhood[][2] = {{-1, -1},
//						{-1, 0},
//						{-1, 1},
//						{0, 1},
//						{1, 1},
//						{1, 0},
//						{1, -1},
//						{0, -1}};
//	std::unique_lock<std::mutex> lck(mtxConstr);
//	constraints = Mat(MAP_SIZE, MAP_SIZE, CV_32FC1, Scalar(0));
//	for(int y = 0; y < MAP_SIZE; y++){
//		for(int x = 0; x < MAP_SIZE; x++){
//			if(countVotes.at<int>(x, y) > 0){
//				constraints.at<float>(x, y) = (float)votes.at<int>(x, y)/countVotes.at<int>(x, y);
//				//cout << x << ":" << y << " = " << (float)votes.at<int>(x, y)/countVotes.at<int>(x, y) << endl;
//			}
//			else{
//				constraints.at<float>(x, y) = 0;
//			}
//		}
//	}
//	for(int y = 0; y < MAP_SIZE; y++){
//		for(int x = 0; x < MAP_SIZE; x++){
//		float maxNhVal = 0;
//		for(int n = 0; n < sizeof(nhood)/sizeof(nhood[0]); n++){
//			int nx = x + nhood[n][0];
//			int ny = y + nhood[n][1];
//			if(nx >= 0 && nx < MAP_SIZE && ny >= 0 && ny < MAP_SIZE){
//				maxNhVal = max(maxNhVal, constraints.at<float>(nx, ny));
//			}
//		}
//		constraints.at<float>(x, y) = min(constraints.at<float>(x, y), maxNhVal);
//		//cout << x << ":" << y << " = " << (float)votes.at<int>(x, y)/countVotes.at<int>(x, y) << endl;
//		}
//	}
//	curTimestamp = nextCurTimestamp;
//	lck.unlock();
//}

std::vector<cv::Mat> Camera::computeMapSegments(cv::Mat curPosImuMapCenter){
//	cout << "Computing map segments" << endl;
	//cout << "curPosMapCenter = " << curPosImuMapCenter << endl;
	//namedWindow("test");
	//mapSegments.clear();
	vector<Mat> ret;

	for(int cam = 0; cam < cameraParams.numCameras; cam++){
		ret.push_back(Mat(cameraParams.numRows, cameraParams.numCols, CV_32SC1, Scalar(-1)));

		//cout << "curPosImuMapCenter*cameraOrigImu[cam]" << endl;
		Mat curPosCameraMapCenterGlobal = cameraParams.imuOrigRobot*curPosImuMapCenter*cameraParams.cameraOrigImu[cam];
		Mat invCameraMatrix = cameraParams.cameraMatrix[cam].inv();

		for(int r = 0; r < cameraParams.numRows; r++){
			for(int c = 0; c < cameraParams.numCols; c++){
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
				Mat pointMapCenter = curPosCameraMapCenterGlobal*pointCam;

//				cout << "pointIm = " << pointIm << endl;
//				cout << "pointCamNN = " << pointCamNN << endl;
//				cout << "s = " << s << endl;
//				cout << "curPosCameraMapCenterGlobal = " << curPosCameraMapCenterGlobal << endl;
//				cout << "pointCam = " << pointCam << endl;
//				cout << "pointMapCenter = " << pointMapCenter << endl;
				//cout << "pointMapCenter.size() =" << pointMapCenter.size() << endl;
				//cout << "mapSegments[cam].size() = " << mapSegments[cam].size() << endl;

				int xSegm = pointMapCenter.at<float>(0)/MAP_RASTER_SIZE + MAP_SIZE/2;
				int ySegm = pointMapCenter.at<float>(1)/MAP_RASTER_SIZE + MAP_SIZE/2;
				//cout << r << ":" << c << " = (" << xSegm << ", " << ySegm << ")" << endl;
				ret[cam].at<int>(r, c) = xSegm*MAP_SIZE + ySegm;
				//cout << "ret[cam].at<int>(r, c) = " << ret[cam].at<int>(r, c) << endl;
				//cout << "End mapSegments[c].at<int>(r, c) =" << endl;
//				waitKey();
			}
		}
//		ofstream headerFile("out.h");
//		headerFile << "#ifndef CAMERA_PROJECT_H" << endl;
//		headerFile << "#define CAMERA_PROJECT_H" << endl << endl;
//		Mat posCameraImu = curPosImuMapCenter*cameraOrigImu[cam];
//		headerFile << "static const float posCameraImuGT[] = {" << endl;
//		for(int r = 0; r < posCameraImu.rows; ++r){
//			for(int c = 0; c < posCameraImu.cols; ++c){
//				headerFile << posCameraImu.at<float>(r, c) << "," << endl;
//			}
//		}
//		headerFile << "};" << endl << endl;
//
//		headerFile << "static const int segIdsGT[] = {" << endl;
//		for(int r = 0; r < ret[cam].rows; ++r){
//			for(int c = 0; c < ret[cam].cols; ++c){
//				headerFile << ret[cam].at<int>(r, c) << "," << endl;
//			}
//		}
//		headerFile << "};" << endl << endl;
//
//		headerFile << "#endif //CAMERA_PROJECT_H" << endl << endl;
//
//		headerFile.close();
//		waitKey();
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

	for(int cam = 0; cam < cameraParams.numCameras; cam++){
		ret.push_back(Mat(cameraParams.numRows, cameraParams.numCols, CV_32SC1, Scalar(-1)));

		//cout << "curPosImuMapCenter*cameraOrigImu[cam]" << endl;
		Mat curPosCameraMapCenterGlobal = cameraParams.imuOrigRobot*curPosImuMapCenter*cameraParams.cameraOrigImu[cam];
		Mat curPosCameraMapCenterImu = curPosImuMapCenter*cameraParams.cameraOrigImu[cam];
		Mat invCameraMatrix = cameraParams.cameraMatrix[cam].inv();

		if(ret[cam].isContinuous() &&
				curPosCameraMapCenterGlobal.isContinuous() &&
				curPosCameraMapCenterImu.isContinuous() &&
				invCameraMatrix.isContinuous())
		{
			reprojectCameraPoints((float*)invCameraMatrix.data,
									(float*)NULL,
									(float*)curPosCameraMapCenterGlobal.data,
									(float*)curPosCameraMapCenterImu.data,
									cameraParams.numRows,
									cameraParams.numCols,
									(int*)ret[cam].data,
									MAP_SIZE,
									MAP_RASTER_SIZE);

		}
	}
	return ret;
}
#endif //NO_CUDA

std::vector<cv::Mat> Camera::computeMapCoords(cv::Mat curPosImuMapCenter){
//	cout << "Computing map segments" << endl;
	//cout << "curPosMapCenter = " << curPosImuMapCenter << endl;
	//namedWindow("test");
	//mapSegments.clear();
	vector<Mat> ret;

	for(int cam = 0; cam < cameraParams.numCameras; cam++){
		ret.push_back(Mat(4, cameraParams.numRows * cameraParams.numCols, CV_32FC1, Scalar(-1)));

		//cout << "curPosImuMapCenter*cameraOrigImu[cam]" << endl;
		Mat curPosCameraMapCenterGlobal = cameraParams.imuOrigRobot*curPosImuMapCenter*cameraParams.cameraOrigImu[cam];
		Mat invCameraMatrix = cameraParams.cameraMatrix[cam].inv();

		for(int r = 0; r < cameraParams.numRows; r++){
			for(int c = 0; c < cameraParams.numCols; c++){
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
				Mat pointMapCenter = curPosCameraMapCenterGlobal*pointCam;

//				cout << "pointIm = " << pointIm << endl;
//				cout << "pointCamNN = " << pointCamNN << endl;
//				cout << "s = " << s << endl;
//				cout << "curPosCameraMapCenterGlobal = " << curPosCameraMapCenterGlobal << endl;
//				cout << "pointCam = " << pointCam << endl;
//				cout << "pointMapCenter = " << pointMapCenter << endl;
				//cout << "pointMapCenter.size() =" << pointMapCenter.size() << endl;
//				cout << "(" << c << ", " << r << ")" << endl;

				pointMapCenter.copyTo(ret[cam].colRange(r * cameraParams.numCols + c, r * cameraParams.numCols + c + 1));
//				cout << "ret[cam].colRange(r * numCols + c, r * numCols + c + 1) = " << ret[cam].colRange(r * numCols + c, r * numCols + c + 1) << endl;
				//cout << "End mapSegments[c].at<int>(r, c) =" << endl;
//				waitKey();
			}
		}
//		ofstream headerFile("out.h");
//		headerFile << "#ifndef CAMERA_PROJECT_H" << endl;
//		headerFile << "#define CAMERA_PROJECT_H" << endl << endl;
//		Mat posCameraImu = curPosImuMapCenter*cameraOrigImu[cam];
//		headerFile << "static const float posCameraImuGT[] = {" << endl;
//		for(int r = 0; r < posCameraImu.rows; ++r){
//			for(int c = 0; c < posCameraImu.cols; ++c){
//				headerFile << posCameraImu.at<float>(r, c) << "," << endl;
//			}
//		}
//		headerFile << "};" << endl << endl;
//
//		headerFile << "static const int segIdsGT[] = {" << endl;
//		for(int r = 0; r < ret[cam].rows; ++r){
//			for(int c = 0; c < ret[cam].cols; ++c){
//				headerFile << ret[cam].at<int>(r, c) << "," << endl;
//			}
//		}
//		headerFile << "};" << endl << endl;
//
//		headerFile << "#endif //CAMERA_PROJECT_H" << endl << endl;
//
//		headerFile.close();
//		waitKey();
	}

//	cout << "End computing map segments" << endl;
	return ret;
}

#ifndef NO_CUDA
std::vector<cv::Mat> Camera::computeMapCoordsGpu(cv::Mat curPosImuMapCenter){
//	cout << "Computing map segments" << endl;
	//namedWindow("test");
	//mapSegments.clear();
	vector<Mat> ret;

	for(int cam = 0; cam < cameraParams.numCameras; cam++){
		ret.push_back(Mat(cameraParams.numRows * cameraParams.numCols, 4, CV_32FC1, Scalar(-1)));

		//cout << "curPosImuMapCenter*cameraOrigImu[cam]" << endl;
		Mat curPosCameraMapCenterGlobal = cameraParams.imuOrigRobot*curPosImuMapCenter*cameraParams.cameraOrigImu[cam];
		Mat curPosCameraMapCenterImu = curPosImuMapCenter*cameraParams.cameraOrigImu[cam];
		Mat invCameraMatrix = cameraParams.cameraMatrix[cam].inv();

		if(ret[cam].isContinuous() &&
				curPosCameraMapCenterGlobal.isContinuous() &&
				curPosCameraMapCenterImu.isContinuous() &&
				invCameraMatrix.isContinuous())
		{
			reprojectCameraPointsCoords((float*)invCameraMatrix.data,
										(float*)NULL,
										(float*)curPosCameraMapCenterGlobal.data,
										(float*)curPosCameraMapCenterImu.data,
										cameraParams.numRows,
										cameraParams.numCols,
										(float*)ret[cam].data,
										MAP_SIZE,
										MAP_RASTER_SIZE);
			ret[cam] = ret[cam].t();
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
					cameraParams.cameraMatrix[cameraInd],
					cameraParams.distCoeffs[cameraInd],
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
						std::vector<cv::Mat>& posesOrigMapCenter,
						std::vector<cv::Mat>& mapCentersOrigGlobal,
						std::vector<std::chrono::high_resolution_clock::time_point>& timestamps,
						std::vector<cv::Mat>& mapMoves,
						std::vector<float>& goalDirsGlobal)
{
	cout << "processDir()" << endl;
	images.clear();
	manualRegionsOnImages.clear();
	mapRegionIdToLabel.clear();
	terrains.clear();
	posesOrigMapCenter.clear();
	mapCentersOrigGlobal.clear();
	timestamps.clear();
	mapMoves.clear();
	goalDirsGlobal.clear();

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
	ifstream goalDirFile(dir.string() + "/goalDirGlobal.data");
	bool includeGoalDirs = true;
	if(!goalDirFile.is_open()){
		cout << "Warning - no goal dir file" << endl;
		includeGoalDirs = false;
	}

	Mat hokuyoAllPointsOrigMapCenter;
	Mat imuPrev, encodersPrev;
	Mat imuOrigGlobal, curPosOrigMapCenter;
	Mat curMapMove = Mat::eye(4, 4, CV_32FC1);
	float curGoalDirGlobal = 0.0;

	std::queue<MovementConstraints::PointsPacket> pointsQueue;
	std::chrono::high_resolution_clock::time_point mapMoveTimestamp(
			std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::milliseconds(0)));

	int hokuyoCurTime;
	hokuyoFile >> hokuyoCurTime;
	int imuCurTime;
	imuFile >> imuCurTime;
	int encodersCurTime;
	encodersFile >> encodersCurTime;
	int goalDirCurTime;
	if(includeGoalDirs){
		goalDirFile >> goalDirCurTime;
	}
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
			if(imuOrigGlobal.empty()){
				imuOrigGlobal = movementConstraints->compOrient(imuPrev);
				curPosOrigMapCenter = Mat::eye(4, 4, CV_32FC1);
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
			curPosOrigMapCenter = movementConstraints->compNewPos(imuPrev, imuCur,
																	encodersPrev, encodersCur,
																	curPosOrigMapCenter,
																	imuOrigGlobal,
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
			cout << "curPosOrigMapCenter = " << endl << curPosOrigMapCenter << endl;
			//cout << "globalPos.inv()*curPos = " << globalPos.inv()*curPos << endl;

//			std::chrono::duration<int,std::milli> durTmp(hokuyoCurTime);
			std::chrono::high_resolution_clock::time_point hokuyoTimestamp(
					std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::milliseconds(hokuyoCurTime)));

			std::mutex mtxPointCloud;

			MovementConstraints::processPointCloud(hokuyoData,
													hokuyoAllPointsOrigMapCenter,
													pointsQueue,
													hokuyoTimestamp,
													hokuyoTimestamp,
													curPosOrigMapCenter,
													mtxPointCloud,
													cameraParams.cameraOrigLaser.front(),
													cameraParams.cameraOrigImu.front(),
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

			cout << "hokuyoAllPointsGlobal.cols = " << hokuyoAllPointsOrigMapCenter.cols << endl;

			std::chrono::high_resolution_clock::time_point curTimestamp(
					std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::milliseconds(hokuyoCurTime)));
//			std::chrono::high_resolution_clock::time_point mapMoveTimestamp(std::chrono::milliseconds(0));

			//move map every 500ms
			if((curTimestamp - mapMoveTimestamp).count() > 500){
				Mat mapMove = curPosOrigMapCenter.inv();

				Mat newPointCloudCoords = mapMove * hokuyoAllPointsOrigMapCenter.rowRange(0, 4);
				newPointCloudCoords.copyTo(hokuyoAllPointsOrigMapCenter.rowRange(0, 4));

				//cout << "Calculating new posMapCenterGlobal" << endl;
				imuOrigGlobal = MovementConstraints::compOrient(imuCur);
				curPosOrigMapCenter = Mat::eye(4, 4, CV_32FC1);

				curMapMove = mapMove * curMapMove;
				mapMoveTimestamp = curTimestamp;
			}

			imuCur.copyTo(imuPrev);
			encodersCur.copyTo(encodersPrev);
			hokuyoFile >> hokuyoCurTime;

			//curRot.copyTo(prevRot);
		}

		if(includeGoalDirs){
			while(goalDirCurTime <= cameraCurTime && !goalDirFile.eof()){
				goalDirFile >> curGoalDirGlobal;
				goalDirFile >> goalDirCurTime;
			}
		}
		goalDirsGlobal.push_back(curGoalDirGlobal);

		Mat terrain = hokuyoAllPointsOrigMapCenter.clone();
		terrain.rowRange(0, 4) = (curPosOrigMapCenter*cameraParams.cameraOrigImu.front()).inv()*hokuyoAllPointsOrigMapCenter.rowRange(0, 4);
		terrains.push_back(terrain);

		posesOrigMapCenter.push_back(curPosOrigMapCenter.clone());

		mapCentersOrigGlobal.push_back(imuOrigGlobal);

		//cout << "Displaying test image from file: " << dir.string() + string("/") + cameraImageFile.string() << endl;
		Mat image = imread(dir.string() + string("/") + cameraImageFile.filename().string());
		//rectangle(image, Point(0, 0), Point(image.cols, 100), Scalar(0, 0, 0), -1);
		images.push_back(image.clone());
		if(image.data == NULL){
			throw "Bad image file";
		}

		std::chrono::high_resolution_clock::time_point timestampImage(
				std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::milliseconds(cameraCurTime)));
		timestamps.push_back(timestampImage);

		mapMoves.push_back(curMapMove.clone());
		curMapMove = Mat::eye(4, 4, CV_32FC1);

		Mat hokuyoAllPointsOrigCamera = (curPosOrigMapCenter*cameraParams.cameraOrigImu.front()).inv()*hokuyoAllPointsOrigMapCenter.rowRange(0, 4);
		//cout << "Computing point projection" << endl;
		vector<Point2f> pointsImage;
		projectPoints(	hokuyoAllPointsOrigCamera.rowRange(0, 3).t(),
						Matx<float, 3, 1>(0, 0, 0),
						Matx<float, 3, 1>(0, 0, 0),
						cameraParams.cameraMatrix.front(),
						cameraParams.distCoeffs.front(),
						pointsImage);
		//cout << "Drawing points" << endl;

//		cout << "Number of points terrain = " << numPts << endl;
//		cout << "mean terrain = " << meanLaser << endl;
//		cout << "covar terrain = " << covarLaser << endl;

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

//		cout << "Number of points image = " << numPix << endl;
//		cout << "mean image = " << meanImage << endl;
//		cout << "covar image = " << covarImage << endl;

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

//		cout << "annotation" << endl;

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
			for(int i = 0; i < cameraParams.labels.size(); i++){
				if(labelText == cameraParams.labels[i]){
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
		{
			double minVal, maxVal;
			minMaxIdx(manualRegionsOnImageCur, &minVal, &maxVal);
			cout << "manualRegionsOnImageCur: min = " << minVal << ", max = " << maxVal << endl;
		}

		mapRegionIdToLabel.push_back(mapRegionIdToLabelCur);
		manualRegionsOnImages.push_back(manualRegionsOnImageCur);
	}

	cout << "End processDir()" << endl;
}

void Camera::learnFromDir(std::vector<boost::filesystem::path> dirs){
	cout << "Learning from dir" << endl;
	//namedWindow("segments");
	//namedWindow("original");
	std::vector<cv::Mat> images;
	std::vector<cv::Mat> manualRegionsOnImages;
	std::vector<std::map<int, int> > mapRegionIdToLabel;
	std::vector<cv::Mat> terrains;
	std::vector<cv::Mat> posesOrigMapCenter;
	std::vector<cv::Mat> mapCentersOrigGlobal;
	std::vector<std::chrono::high_resolution_clock::time_point> timestamps;
	std::vector<cv::Mat> mapMoves;
	std::vector<float> goalDirsGlobal;

	for(int d = 0; d < dirs.size(); d++){
		std::vector<cv::Mat> tmpImages;
		std::vector<cv::Mat> tmpManualRegionsOnImages;
		std::vector<std::map<int, int> > tmpMapRegionIdToLabel;
		std::vector<cv::Mat> tmpTerrains;
		std::vector<cv::Mat> tmpPosesOrigMapCenter;
		std::vector<cv::Mat> tmpMapCentersOrigGlobal;
		std::vector<std::chrono::high_resolution_clock::time_point> tmpTimestamps;
		std::vector<cv::Mat> tmpMapMoves;
		std::vector<float> tmpGoalDirsGlobal;

		processDir(	dirs[d],
					tmpImages,
					tmpManualRegionsOnImages,
					tmpMapRegionIdToLabel,
					tmpTerrains,
					tmpPosesOrigMapCenter,
					tmpMapCentersOrigGlobal,
					tmpTimestamps,
					tmpMapMoves,
					tmpGoalDirsGlobal);


		images.insert(images.end(), tmpImages.begin(), tmpImages.end());
		manualRegionsOnImages.insert(manualRegionsOnImages.end(), tmpManualRegionsOnImages.begin(), tmpManualRegionsOnImages.end());
		mapRegionIdToLabel.insert(mapRegionIdToLabel.end(), tmpMapRegionIdToLabel.begin(), tmpMapRegionIdToLabel.end());
		terrains.insert(terrains.end(), tmpTerrains.begin(), tmpTerrains.end());
		posesOrigMapCenter.insert(posesOrigMapCenter.end(), tmpPosesOrigMapCenter.begin(), tmpPosesOrigMapCenter.end());
		mapCentersOrigGlobal.insert(mapCentersOrigGlobal.end(), tmpMapCentersOrigGlobal.begin(), tmpMapCentersOrigGlobal.end());
		timestamps.insert(timestamps.end(), tmpTimestamps.begin(), tmpTimestamps.end());
		mapMoves.insert(mapMoves.end(), tmpMapMoves.begin(), tmpMapMoves.end());
		goalDirsGlobal.insert(goalDirsGlobal.end(), tmpGoalDirsGlobal.begin(), tmpGoalDirsGlobal.end());
	}

	cout << "images.size() = " << images.size() << endl << "manualRegionsOnImages.size() = " << manualRegionsOnImages.size() << endl;

	vector<Entry> dataset;


	Mat pixelCoordsAll;
	vector<Mat> classResultsAll;
	for(int l = 0; l < cameraParams.labels.size(); ++l){
		classResultsAll.push_back(Mat());
	}
	Mat manualLabelsAll;
	Mat pixelColorsAll;

	std::queue<ClassResult> classResultsHistDir;

	for(int i = 0; i < images.size(); i++){
		cout << "Segmenting" << endl;
#ifdef NO_CUDA
		vector<Mat> pixelCoordsVec = computeMapCoords(posesOrigMapCenter[i]);
#else
		vector<Mat> pixelCoordsVec = computeMapCoordsGpu(posesOrigMapCenter[i]);
#endif
		Mat pixelCoords = pixelCoordsVec.front();

		Mat autoRegionsOnImage(images[i].rows, images[i].cols, CV_32SC1);

		for(int r = 0; r < images[i].rows; ++r){
			for(int c = 0; c < images[i].cols; ++c){
				int xSegm = pixelCoords.at<float>(0, r * images[i].cols + c)/MAP_RASTER_SIZE + MAP_SIZE/2;
				int ySegm = pixelCoords.at<float>(1, r * images[i].cols + c)/MAP_RASTER_SIZE + MAP_SIZE/2;
//				cout << r << ":" << c << " = (" << xSegm << ", " << ySegm << ")" << endl;
//				cout << "(" << pixelCoords.at<float>(0, r * images[i].cols + c) << ", " << pixelCoords.at<float>(1, r * images[i].cols + c)
//						<< " : (" << xSegm << ", " << ySegm << ")" << endl;
				autoRegionsOnImage.at<int>(r, c) = xSegm*MAP_SIZE + ySegm;
			}
		}

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
																		cameraParams.maskIgnore.front(),
																		cameraParams.entryWeightThreshold);

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

	cout << "crossValidate = " << cameraParams.crossValidate << endl;
	if(cameraParams.crossValidate){
		hierClassifiers.front()->crossValidateSVMs(dataset);
	}
	hierClassifiers.front()->train(dataset, cameraParams.labels.size());

	if(cameraParams.cacheSaveEnabled){
		saveCache("cache/cameraCache");
	}
}

void Camera::classifyFromDir(std::vector<boost::filesystem::path> dirs){
	static const bool compareWithExt = false;
	static const bool estimatePgmParams = false;
	static const bool computeControlError = true;

	cout << "Classifying" << endl;
	for(int l = 0; l < cameraParams.labels.size(); l++){
		namedWindow(cameraParams.labels[l]);
	}
	namedWindow("segments");
	namedWindow("labeled");
	namedWindow("classified");

	//create a window for visualization
	viz::Viz3d win("camera visualization");
	init3DVis(win);

	vector<vector<int> > classResultsPix(cameraParams.labels.size(), vector<int>(cameraParams.labels.size(), 0));
	vector<vector<int> > classResultsSeg(cameraParams.labels.size(), vector<int>(cameraParams.labels.size(), 0));

	std::vector<cv::Mat> images;
	std::vector<cv::Mat> manualRegionsOnImages;
	std::vector<std::map<int, int> > mapRegionIdToLabel;
	std::vector<cv::Mat> terrains;
	std::vector<cv::Mat> posesOrigMapCenter;
	std::vector<cv::Mat> mapCentersOrigGlobal;
	std::vector<std::chrono::high_resolution_clock::time_point> timestamps;
	std::vector<cv::Mat> mapMoves;
	std::vector<float> goalDirsGlobal;

	for(int d = 0; d < dirs.size(); d++){
		std::vector<cv::Mat> tmpImages;
		std::vector<cv::Mat> tmpManualRegionsOnImages;
		std::vector<std::map<int, int> > tmpMapRegionIdToLabel;
		std::vector<cv::Mat> tmpTerrains;
		std::vector<cv::Mat> tmpPosesOrigMapCenter;
		std::vector<cv::Mat> tmpMapCentersOrigGlobal;
		std::vector<std::chrono::high_resolution_clock::time_point> tmpTimestamps;
		std::vector<cv::Mat> tmpMapMoves;
		std::vector<float> tmpGoalDirsGlobal;

		processDir(	dirs[d],
					tmpImages,
					tmpManualRegionsOnImages,
					tmpMapRegionIdToLabel,
					tmpTerrains,
					tmpPosesOrigMapCenter,
					tmpMapCentersOrigGlobal,
					tmpTimestamps,
					tmpMapMoves,
					tmpGoalDirsGlobal);


		images.insert(images.end(), tmpImages.begin(), tmpImages.end());
		manualRegionsOnImages.insert(manualRegionsOnImages.end(), tmpManualRegionsOnImages.begin(), tmpManualRegionsOnImages.end());
		mapRegionIdToLabel.insert(mapRegionIdToLabel.end(), tmpMapRegionIdToLabel.begin(), tmpMapRegionIdToLabel.end());
		terrains.insert(terrains.end(), tmpTerrains.begin(), tmpTerrains.end());
		posesOrigMapCenter.insert(posesOrigMapCenter.end(), tmpPosesOrigMapCenter.begin(), tmpPosesOrigMapCenter.end());
		mapCentersOrigGlobal.insert(mapCentersOrigGlobal.end(), tmpMapCentersOrigGlobal.begin(), tmpMapCentersOrigGlobal.end());
		timestamps.insert(timestamps.end(), tmpTimestamps.begin(), tmpTimestamps.end());
		mapMoves.insert(mapMoves.end(), tmpMapMoves.begin(), tmpMapMoves.end());
		goalDirsGlobal.insert(goalDirsGlobal.end(), tmpGoalDirsGlobal.begin(), tmpGoalDirsGlobal.end());
	}

	cout << "images.size() = " << images.size() << endl << "manualRegionsOnImages.size() = " << manualRegionsOnImages.size() << endl;

	//read data for comparison with external system
	vector<vector<int> > compResVal;
	vector<vector<int> > compResSegId;

	if(compareWithExt){
		ifstream compResFile("res.log");
		if(!compResFile.is_open()){
			throw "Error could not open file for compare with external system";
		}
		char tmp;
		int imNum;
		compResFile >> tmp;
		while(tmp == 'i' && !compResFile.eof() && !compResFile.fail()){
			compResFile >> imNum;
			cout << "Compare image " << imNum << endl;
			compResVal.push_back(vector<int>());
			compResSegId.push_back(vector<int>());

			compResFile >> tmp;
			while(tmp == 'e' && !compResFile.eof() && !compResFile.fail()){
				int segId;
				compResFile >> segId;
				compResSegId.back().push_back(segId);

				int val;
				compResFile >> val;
				compResVal.back().push_back(val);

				compResFile >> tmp;
			}

		}
	}

//	vector<Entry> dataset;
	ofstream dataCrfFile("log/dataCrf.log");
	dataCrfFile.precision(15);
	bool headerWritten = false;

	Mat pixelCoordsAll;
	vector<Mat> classResultsAll;
	for(int l = 0; l < cameraParams.labels.size(); ++l){
		classResultsAll.push_back(Mat());
	}
	Mat manualLabelsAll;
	Mat pixelColorsAll;

	std::queue<ClassResult> classResultsHistDir;

	vector<Pgm> pgmsParamEst;
	vector<vector<double> > varValsParamEst;
	vector<vector<double> > obsVecsParamEst;

	float controlError = 0.0;
	float maxControlError = 0.0;
	int cntRelevantControlError = 0;

	for(int i = 0; i < images.size(); i++){
		cout << "Segmenting" << endl;
#ifdef NO_CUDA
		vector<Mat> pixelCoordsVec = computeMapCoords(posesOrigMapCenter[i]);
#else
		vector<Mat> pixelCoordsVec = computeMapCoordsGpu(posesOrigMapCenter[i]);
#endif
		Mat pixelCoords = pixelCoordsVec.front();

		Mat autoRegionsOnImage(images[i].rows, images[i].cols, CV_32SC1);

		for(int r = 0; r < images[i].rows; ++r){
			for(int c = 0; c < images[i].cols; ++c){
				int xSegm = pixelCoords.at<float>(0, r * images[i].cols + c)/MAP_RASTER_SIZE + MAP_SIZE/2;
				int ySegm = pixelCoords.at<float>(1, r * images[i].cols + c)/MAP_RASTER_SIZE + MAP_SIZE/2;
//				cout << r << ":" << c << " = (" << xSegm << ", " << ySegm << ")" << endl;
//				cout << "(" << pixelCoords.at<float>(0, r * images[i].cols + c) << ", " << pixelCoords.at<float>(1, r * images[i].cols + c)
//						<< " : (" << xSegm << ", " << ySegm << ")" << endl;
				autoRegionsOnImage.at<int>(r, c) = xSegm*MAP_SIZE + ySegm;
			}
		}

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
																		cameraParams.maskIgnore.front(),
																		cameraParams.entryWeightThreshold);


		for(int e = 0; e < newData.size(); e++){
			if(mapRegionIdToLabel[i].count(assignedManualId[newData[e].imageId]) > 0){
				newData[e].label = mapRegionIdToLabel[i][assignedManualId[newData[e].imageId]];
//				dataset.push_back(newData[e]);
			}
		}


		vector<Scalar> colors;
		colors.push_back(Scalar(0, 255, 0));	//grass - green
		colors.push_back(Scalar(0, 0, 255));	//wood - red
		colors.push_back(Scalar(0, 255, 255));	//yellow - ceramic
		colors.push_back(Scalar(255, 0, 0));	//blue - asphalt

		Mat coloredOriginalLabeled = images[i].clone();

		for(int e = 0; e < newData.size(); e++){
			if(mapRegionIdToLabel[i].count(assignedManualId[newData[e].imageId]) > 0){
				int label = newData[e].label;
				if(label < 0 || label > 1){
					throw "Bad label";
				}
				coloredOriginalLabeled.setTo(colors[label], autoRegionsOnImage == newData[e].imageId);
			}
		}
		Mat visualization = coloredOriginalLabeled * 0.25 + images[i] * 0.75;
		//ret.setTo(Scalar(0, 0, 0), maskIgnore.front() != 0);

		imshow("labeled", visualization);
		imshow("segments", hierClassifiers.front()->colorSegments(autoRegionsOnImage));

		cout << "Classifing" << endl;
		vector<Mat> classificationResult = hierClassifiers.front()->classify(images[i],
																			terrains[i],
																			autoRegionsOnImage,
																			cameraParams.maskIgnore.front(),
																			cameraParams.entryWeightThreshold);
		cout << "End classifing" << endl;

		//display current classification results
		for(int l = 0; l < cameraParams.labels.size(); l++){
			double minVal, maxVal;
			minMaxIdx(classificationResult[l], &minVal, &maxVal);
			cout << cameraParams.labels[l] << ", min = " << minVal << ", max = " << maxVal << endl;
			imshow(cameraParams.labels[l], classificationResult[l]);
		}

		//display current classified image
		Mat coloredOriginalClassified = images[i].clone();
		Mat bestLabels(images[i].rows, images[i].cols, CV_32SC1, Scalar(-1));
		Mat bestScore(images[i].rows, images[i].cols, CV_32FC1, Scalar(-1));
		for(int l = 0; l < cameraParams.labels.size(); l++){
			Mat cmp;
			compare(bestScore, classificationResult[l], cmp, CMP_LE);
			bestLabels.setTo(l, cmp);
			bestScore = max(bestScore, classificationResult[l]);
		}
		for(int l = 0; l < cameraParams.labels.size(); l++){
			coloredOriginalClassified.setTo(colors[l], bestLabels == l);
		}
		imshow("classified", coloredOriginalClassified * 0.25 + images[i] * 0.75);

		//display external classification results
		if(compareWithExt){
			Mat coloredOriginalCompClassified = images[i].clone();
			Mat labelsComp(images[i].rows, images[i].cols, CV_32SC1, Scalar(-1));

			for(int comp = 0; comp < compResVal[i].size(); ++comp){
				Mat cmp;
				compare(compResSegId[i][comp], autoRegionsOnImage, cmp, CMP_EQ);
				labelsComp.setTo(compResVal[i][comp], cmp);
			}
			for(int l = 0; l < cameraParams.labels.size(); l++){
				coloredOriginalCompClassified.setTo(colors[l], labelsComp == l);
			}

			imshow("classified comp", coloredOriginalCompClassified * 0.25 + images[i] * 0.75);
		}

		//merge with previous pixel data

		//generating labeled image
		Mat manualLabelsOnImage(manualRegionsOnImages[i].rows, manualRegionsOnImages[i].cols, CV_32SC1, Scalar(-1));
		for(map<int, int>::iterator it = mapRegionIdToLabel[i].begin(); it != mapRegionIdToLabel[i].end(); ++it){
			manualLabelsOnImage.setTo(it->second, manualRegionsOnImages[i] == it->first);
		}

		updatePixelData(pixelCoordsAll,
						classResultsAll,
						manualLabelsAll,
						pixelColorsAll,
						classResultsHistDir,
						mapMoves[i],
						timestamps[i],
						pixelCoords,
						classificationResult,
						images[i],
						manualLabelsOnImage);

		cout << "Num pixels = " << pixelCoordsAll.cols << endl;

		//assign manual label for each observed segment
		cout << "assigning labels" << endl;
		Mat segmentManualLabels = assignSegmentLabels(manualLabelsAll, pixelCoordsAll);
//		cout << "segmentManualLabels = " << segmentManualLabels << endl;

		Mat laserPointCloudOrigRobotMapCenter = terrains[i].clone();
		laserPointCloudOrigRobotMapCenter.rowRange(0, 4) = cameraParams.imuOrigRobot *
															posesOrigMapCenter[i] *
															cameraParams.cameraOrigImu.front() *
															laserPointCloudOrigRobotMapCenter.rowRange(0, 4);
		//run inference
		cout << "running inference" << endl;

		Mat inferResults(1, pixelCoordsAll.cols, CV_32SC1);
		Mat segmentResults;

		if(cameraParams.inferenceEnabled){
			std::vector<cv::Mat> segmentPriors;
			std::vector<cv::Mat> segmentFeats;
			std::vector<int> segmentPixelCount;
			Pgm pgm;
			std::map<int, int> segIdToVarClusterId;
			std::map<int, int> segIdToRandVarId;
			std::vector<double> obsVec;
	//		Mat segmentMask(segmentManualLabels.size(), CV_32SC1, Scalar(0));
	//		segmentMask.setTo(Scalar(1), segmentManualLabels >= 0);

			cout << "prepare segment info" << endl;
			prepareSegmentInfo(segmentPriors,
								segmentFeats,
								segmentPixelCount,
								pixelCoordsAll,
								pixelColorsAll,
								classResultsAll,
								laserPointCloudOrigRobotMapCenter,
								segmentManualLabels >= 0);

			cout << "construct pgm" << endl;
			constructPgm(pgm,
						segIdToVarClusterId,
						segIdToRandVarId,
						obsVec,
						segmentPriors,
						segmentFeats,
						segmentPixelCount);

			cout << "infer terrain labels" << endl;
			segmentResults = inferTerrainLabels(pgm,
												obsVec,
												segIdToVarClusterId);

			cout << "infer results" << endl;
			for(int d = 0; d < manualLabelsAll.cols; ++d){
				int xSegm = pixelCoordsAll.at<float>(0, d)/MAP_RASTER_SIZE + MAP_SIZE/2;
				int ySegm = pixelCoordsAll.at<float>(1, d)/MAP_RASTER_SIZE + MAP_SIZE/2;

				inferResults.at<int>(d) = segmentResults.at<int>(xSegm, ySegm);
			}

			if(estimatePgmParams){
				pgmsParamEst.push_back(pgm);
				//Add varVals
				std::vector<double> varVals(segIdToRandVarId.size(), 0.0);
				for(int mapX = 0; mapX < MAP_SIZE; ++mapX){
					for(int mapY = 0; mapY < MAP_SIZE; ++mapY){
						int segId = mapX * MAP_SIZE + mapY;
						if(segIdToRandVarId.count(segId) > 0){
							int rvId = segIdToRandVarId.at(segId);
							varVals[rvId] = segmentManualLabels.at<int>(mapX, mapY);
							if(varVals[rvId] < 0){
								throw "Error varVals[rvId] < 0";
							}
						}
					}
				}
				varValsParamEst.push_back(varVals);
	//			cout << "varVals = " << varVals << endl;
				obsVecsParamEst.push_back(obsVec);
	//			cout << "obsVec = " << obsVec << endl;
			}
			else{
				//Pgm structure no longer needed - deleting
				pgm.deleteContents();
				for(int f = 0; f < pgm.feats().size(); ++f){
					delete pgm.feats()[f];
				}
			}
		}
		else{
			//computing labeling without inference, basing on classifier results voting
			for(int d = 0; d < pixelCoordsAll.cols; ++d){
				int bestLabelInd = -1;
				float bestLabelScore = 0;
				for(int l = 0; l < classResultsAll.size(); ++l){
					if(classResultsAll[l].at<float>(0, d) > bestLabelScore){
						bestLabelInd = l;
						bestLabelScore = classResultsAll[l].at<float>(0, d);
					}
				}
				inferResults.at<int>(d) = bestLabelInd;
			}

			//assign result for each observed segment - temporary, should be obtained during inference
			cout << "assigning results" << endl;
			segmentResults = assignSegmentLabels(inferResults, pixelCoordsAll);
	//		cout << "segmentResults = " << segmentResults << endl;
		}

		float bestDirLocalMap = 0;
		float goalDirLocalMap = 0;
		float bestDirLocalMapManual = 0;
		float goalDirLocalMapManual = 0;
		bool stopFlag = false;
		if(computeControlError){
			if(cameraParams.debugLevel >= 1){
				cout << "computing control error" << endl;
			}

			computeBestDirLocalMap(segmentResults,
									posesOrigMapCenter[i],
									mapCentersOrigGlobal[i],
									goalDirsGlobal[i],
									bestDirLocalMap,
									goalDirLocalMap);

			cout << "goalDir global = " <<  goalDirsGlobal[i] << endl;
			cout << "goalDir = " << goalDirLocalMap << endl;
			cout << "bestDir = " << bestDirLocalMap << endl;

			computeBestDirLocalMap(segmentManualLabels,
									posesOrigMapCenter[i],
									mapCentersOrigGlobal[i],
									goalDirsGlobal[i],
									bestDirLocalMapManual,
									goalDirLocalMapManual);

			cout << "goalDir global = " <<  goalDirsGlobal[i] << endl;
			cout << "goalDir = " << goalDirLocalMap << endl;
			cout << "bestDir = " << bestDirLocalMap << endl;
			cout << "bestDir manual = " << bestDirLocalMapManual << endl;

			float curControlError = min(fabs(bestDirLocalMap - bestDirLocalMapManual),
									fabs(fabs(bestDirLocalMap - bestDirLocalMapManual) - 360));
			cout << "curControlError = " << curControlError << endl;
			if(curControlError >= 30){
//				stopFlag = true;
				cntRelevantControlError++;
			}
			maxControlError = max(maxControlError, curControlError);
			controlError += curControlError;
		}

		//display results after inference
		Mat coloredOriginalResSeg = images[i].clone();
		for(int mapX = 0; mapX < MAP_SIZE; ++mapX){
			for(int mapY = 0; mapY < MAP_SIZE; ++mapY){
				int segId = mapX*MAP_SIZE + mapY;
				int lab = segmentResults.at<int>(mapX, mapY);
				if(lab >= 0){
					coloredOriginalResSeg.setTo(colors[lab], autoRegionsOnImage == segId);
				}
			}
		}
		imshow("results seg", coloredOriginalResSeg * 0.25 + images[i] * 0.75);

		//display manual labels from all data
		Mat coloredOriginalManualSeg = images[i].clone();
		for(int mapX = 0; mapX < MAP_SIZE; ++mapX){
			for(int mapY = 0; mapY < MAP_SIZE; ++mapY){
				int segId = mapX*MAP_SIZE + mapY;
				int lab = segmentManualLabels.at<int>(mapX, mapY);
				if(lab >= 0){
					coloredOriginalManualSeg.setTo(colors[lab], autoRegionsOnImage == segId);
				}
			}
		}
		imshow("manual seg", coloredOriginalManualSeg * 0.25 + images[i] * 0.75);

		//count current results
		vector<vector<int> > curClassResultsPix(cameraParams.labels.size(), vector<int>(cameraParams.labels.size(), 0));
		vector<vector<int> > curClassResultsSeg(cameraParams.labels.size(), vector<int>(cameraParams.labels.size(), 0));

		//pixel-wise
		for(int d = 0; d < manualLabelsAll.cols; ++d){
			int res = inferResults.at<int>(d);
			int gt = manualLabelsAll.at<int>(d);
			if(res >= 0 && gt >= 0){
				curClassResultsPix[gt][res]++;
			}
		}

		//segment-wise
		for(int mapX = 0; mapX < MAP_SIZE; ++mapX){
			for(int mapY = 0; mapY < MAP_SIZE; ++mapY){
				int res = segmentResults.at<int>(mapX, mapY);
				int gt = segmentManualLabels.at<int>(mapX, mapY);
				if(res >= 0 && gt >= 0)				{
					curClassResultsSeg[gt][res]++;
				}
			}
		}

		//export data for CRF - header
		if(headerWritten == false){
			dataCrfFile << cameraParams.labels.size() << " " << newData.front().descriptor.cols << endl;
			headerWritten = true;
		}

		dataCrfFile << "i " << i << endl;

		//export data for CRF - data
		set<int> counted;
		for(int r = 0; r < images[i].rows; r++){
			for(int c = 0; c < images[i].cols; c++){
				int pred = bestLabels.at<int>(r, c);
				if(pred >= 0){
					if(mapRegionIdToLabel[i].count(assignedManualId[autoRegionsOnImage.at<int>(r, c)]) > 0){
						if(counted.count(autoRegionsOnImage.at<int>(r, c)) == 0){
							int groundTrue = mapRegionIdToLabel[i][assignedManualId[autoRegionsOnImage.at<int>(r, c)]];

							int newDataIdx = -1;
							for(int e = 0; e < newData.size(); e++){
								if(newData[e].imageId == autoRegionsOnImage.at<int>(r, c)){
									newDataIdx = e;
									break;
								}
							}

							if(newDataIdx >= 0){
								dataCrfFile << "e " << newDataIdx << " " << autoRegionsOnImage.at<int>(r, c) << " " <<
										groundTrue << " " << newData[newDataIdx].weight << " ";
								for(int l = 0; l < cameraParams.labels.size(); ++l){
									dataCrfFile << classificationResult[l].at<float>(r, c) << " ";
								}
								Mat descNorm = hierClassifiers.front()->normalizeDesc(newData[newDataIdx].descriptor);
								for(int col = 0; col < descNorm.cols; ++col){
									dataCrfFile << descNorm.at<float>(0, col) << " ";
								}
								dataCrfFile << endl;
							}

							counted.insert(autoRegionsOnImage.at<int>(r, c));
						}
					}
				}
			}
		}

		//add current results to overall results
		for(int t = 0; t < cameraParams.labels.size(); t++){
			for(int p = 0; p < cameraParams.labels.size(); p++){
				classResultsPix[t][p] += curClassResultsPix[t][p];
				classResultsSeg[t][p] += curClassResultsSeg[t][p];
			}
		}

		cout << "Current frame pixel results: " << endl;
		for(int t = 0; t < cameraParams.labels.size(); t++){
			cout << "true = " << t << ": ";
			for(int p = 0; p < cameraParams.labels.size(); p++){
				cout << curClassResultsPix[t][p] << ", ";
			}
			cout << endl;
		}

		cout << "Current frame segment results: " << endl;
		for(int t = 0; t < cameraParams.labels.size(); t++){
			cout << "true = " << t << ": ";
			for(int p = 0; p < cameraParams.labels.size(); p++){
				cout << curClassResultsSeg[t][p] << ", ";
			}
			cout << endl;
		}

		draw3DVis(win,
				pixelCoordsAll,
				pixelColorsAll,
				posesOrigMapCenter[i],
				segmentResults,
				laserPointCloudOrigRobotMapCenter,
				segmentManualLabels,
				goalDirLocalMap,
				bestDirLocalMap,
				bestDirLocalMapManual,
				stopFlag);
//		waitKey(1000);
	}

	dataCrfFile.close();

	cout << "General pixel results: " << endl;
	for(int t = 0; t < cameraParams.labels.size(); t++){
		cout << "true = " << t << ": ";
		for(int p = 0; p < cameraParams.labels.size(); p++){
			cout << classResultsPix[t][p] << ", ";
		}
		cout << endl;
	}

	cout << "General segment results: " << endl;
	for(int t = 0; t < cameraParams.labels.size(); t++){
		cout << "true = " << t << ": ";
		for(int p = 0; p < cameraParams.labels.size(); p++){
			cout << classResultsSeg[t][p] << ", ";
		}
		cout << endl;
	}

	if(computeControlError){
		cout << "Overall control error = " << controlError << endl;
		cout << "Max control error = " << maxControlError << endl;
		cout << "Counter relevant control error = " << cntRelevantControlError << endl;
	}

	if(estimatePgmParams){
		ParamEst paramEst;
		paramEst.estimateParams(pgmsParamEst,
								varValsParamEst,
								obsVecsParamEst);

		for(int p = 0; p < pgmsParamEst.size(); ++p){
			pgmsParamEst[p].deleteContents();
			for(int f = 0; f < pgmsParamEst[p].feats().size(); ++f){
				delete pgmsParamEst[p].feats()[f];
			}
		}
	}

	cout << "End classifying" << endl;
}


//Run as separate thread
void Camera::run(){

	try{
		while(runThread){
			if(cameraParams.debugLevel >= 1){
				cout << "Camera run" << endl;
			}
			std::chrono::high_resolution_clock::time_point timeBegin;
			std::chrono::high_resolution_clock::time_point timeEndMapCoords;
			std::chrono::high_resolution_clock::time_point timeEndClassification;
			std::chrono::high_resolution_clock::time_point timeEndUpdate;
			Mat curPosOrigMapCenter;

			std::unique_lock<std::mutex> lckClassRes(mtxClassResults);

			std::chrono::high_resolution_clock::time_point curTimestamp = std::chrono::high_resolution_clock::now();
			Mat pointCloudOrigMapCenter = movementConstraints->getPointCloud(curPosOrigMapCenter);

			mapMoveSinceGetPointCloud = Mat::eye(4, 4, CV_32FC1);

			lckClassRes.unlock();

			if(!curPosOrigMapCenter.empty()){
				timeBegin = std::chrono::high_resolution_clock::now();
				vector<Mat> cameraData = this->getData();
				if(cameraParams.debugLevel >= 1){
					cout << "Computing map coords" << endl;
				}
	#ifdef NO_CUDA
				vector<Mat> pixelCoordsVec = computeMapCoords(curPosOrigMapCenter);
	#else
				vector<Mat> pixelCoordsVec = computeMapCoordsGpu(curPosOrigMapCenter);
	#endif
				timeEndMapCoords = std::chrono::high_resolution_clock::now();
				for(int cam = 0; cam < cameraParams.numCameras; cam++){
					if(cameras[cam].isOpened()){
						if(!cameraData[cam].empty()){
							if(cameraParams.debugLevel >= 1){
								cout << "camera " << cam << endl;
							}
							Mat pointCloudOrigCamera;
							Mat curPointCloudOrigRobotMapCenter;
							if(!pointCloudOrigMapCenter.empty()){
								pointCloudOrigCamera = Mat(pointCloudOrigMapCenter.rows, pointCloudOrigMapCenter.cols, CV_32FC1);
								pointCloudOrigMapCenter.rowRange(4, 6).copyTo(pointCloudOrigCamera.rowRange(4, 6));
								Mat tmpPointCoords = (curPosOrigMapCenter * cameraParams.cameraOrigImu[cam]).inv() * pointCloudOrigMapCenter.rowRange(0, 4);
								tmpPointCoords.copyTo(pointCloudOrigCamera.rowRange(0, 4));

								curPointCloudOrigRobotMapCenter = Mat(pointCloudOrigMapCenter.rows, pointCloudOrigMapCenter.cols, CV_32FC1);
								pointCloudOrigMapCenter.rowRange(4, 6).copyTo(curPointCloudOrigRobotMapCenter.rowRange(4, 6));
								tmpPointCoords = cameraParams.imuOrigRobot * pointCloudOrigMapCenter.rowRange(0, 4);
								tmpPointCoords.copyTo(curPointCloudOrigRobotMapCenter.rowRange(0, 4));
							}

//							cout << "pixelCoordsVec.size() = " << pixelCoordsVec.size() << endl;
							Mat pixelCoords = pixelCoordsVec[cam];
//							cout << "pixelCoords.size() = " << pixelCoords.size() << endl;
//							cout << "cameraData[c].size() = " << cameraData[cam].size() << endl;
							Mat mapSegments(cameraData[cam].rows, cameraData[cam].cols, CV_32SC1);

//							cout << "computing map segments" << endl;
							for(int row = 0; row < cameraData[cam].rows; ++row){
								for(int col = 0; col < cameraData[cam].cols; ++col){
									int xSegm = pixelCoords.at<float>(0, row * cameraData[cam].cols + col)/MAP_RASTER_SIZE + MAP_SIZE/2;
									int ySegm = pixelCoords.at<float>(1, row * cameraData[cam].cols + col)/MAP_RASTER_SIZE + MAP_SIZE/2;
//									cout << row << ":" << col << " = (" << xSegm << ", " << ySegm << ")" << endl;
					//				cout << "(" << pixelCoords.at<float>(0, r * images[i].cols + c) << ", " << pixelCoords.at<float>(1, r * images[i].cols + c)
					//						<< " : (" << xSegm << ", " << ySegm << ")" << endl;
									mapSegments.at<int>(row, col) = xSegm*MAP_SIZE + ySegm;
								}
							}

							if(cameraParams.debugLevel >= 1){
								cout << "Classification" << endl;
							}

							vector<Mat> classRes = hierClassifiers[cam]->classify(cameraData[cam],
																				pointCloudOrigCamera,
																				mapSegments,
																				cameraParams.maskIgnore[cam],
																				cameraParams.entryWeightThreshold);
							timeEndClassification = std::chrono::high_resolution_clock::now();

							if(cameraParams.debugLevel >= 1){
								cout << "End classification" << endl;
							}

							Mat bestLabels(cameraParams.numRows, cameraParams.numCols, CV_32SC1, Scalar(-1));
							Mat bestScore(cameraParams.numRows, cameraParams.numCols, CV_32FC1, Scalar(-1));
							for(int l = 0; l < cameraParams.labels.size(); l++){
								Mat cmp;
								//cout << "classRes[l].size() = " << classRes[l].size() << endl;
								//cout << "bestScore.rows = " << bestScore.rows << endl;
								compare(bestScore, classRes[l], cmp, CMP_LE);
								//cout << "bestLabels.size() = " << bestLabels.size() << endl;
								bestLabels.setTo(l, cmp);
								//cout << "max" << endl;
								bestScore = max(bestScore, classRes[l]);
							}
							classifiedImage[cam] = bestLabels;
							//classifiedImage[c] = Mat(numRows, numCols, CV_32SC1, Scalar(0));
							if(cameraParams.debugLevel >= 1){
								cout << "Copying classified image" << endl;
							}
							std::unique_lock<std::mutex> lck(mtxClassIm);
							classifiedImage[cam].copyTo(sharedClassifiedImage);
							cameraData[cam].copyTo(sharedOriginalImage);
							lck.unlock();


							if(cameraParams.debugLevel >= 1){
								cout << "Merging with previous pixel data" << endl;
							}

							lckClassRes.lock();

							//Ensure correct pixelCoords values if map moved since getPointCloud!!!
							if(timestampMap > curTimestamp){
								if(cameraParams.debugLevel >= 2){
									cout << endl << "Map moved since getPointCloud!!!" << endl << endl;
								}
								pixelCoords = mapMoveSinceGetPointCloud * pixelCoords;
							}

							//merge with previous pixel data

							//initialize if empty
							if(classResultsMap.empty()){
								//ensure that every Mat in vector points to a separate data container
								for(int l = 0; l < cameraParams.labels.size(); ++l){
									classResultsMap.push_back(Mat());
								}
							}

							Mat manualLabelsMap;
							updatePixelData(pixelCoordsMapOrigRobotMapCenter,
											classResultsMap,
											manualLabelsMap,
											pixelColorsMap,
											classResultsHistMap,
											Mat::eye(4, 4, CV_32FC1),
											curTimestamp,
											pixelCoords,
											classRes,
											cameraData[cam]);

							pointCloudMapOrigRobotMapCenter = curPointCloudOrigRobotMapCenter.clone();

							cout << "Num pixels = " << pixelCoordsMapOrigRobotMapCenter.cols << endl;

							lckClassRes.unlock();
						}
					}
				}
//				computeConstraints(nextCurTimestamp);
				timeEndUpdate = std::chrono::high_resolution_clock::now();
			}
			if(cameraParams.debugLevel >= 1){
				cout << "Map segments time: " << std::chrono::duration_cast<std::chrono::milliseconds>(timeEndMapCoords - timeBegin).count() << " ms" << endl;
				cout << "Classification time: " << std::chrono::duration_cast<std::chrono::milliseconds>(timeEndClassification - timeEndMapCoords).count() << " ms" << endl;
				cout << "Update time: " << std::chrono::duration_cast<std::chrono::milliseconds>(timeEndUpdate - timeEndClassification).count() << " ms" << endl;
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

	if(settings->QueryIntAttribute("number", &cameraParams.numCameras) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of cameras";
	}
	if(settings->QueryIntAttribute("rows", &cameraParams.numRows) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of rows";
	}
	if(settings->QueryIntAttribute("cols", &cameraParams.numCols) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of cols";
	}
	if(settings->QueryIntAttribute("entryWeightThreshold", &cameraParams.entryWeightThreshold) != TIXML_SUCCESS){
		throw "Bad settings file - wrong entryWeightThreshold";
	}
	if(settings->QueryIntAttribute("pixelsTimeout", &cameraParams.pixelsTimeout) != TIXML_SUCCESS){
		throw "Bad settings file - wrong pixelsTimeout";
	}
	if(settings->QueryIntAttribute("debug", &cameraParams.debugLevel) != TIXML_SUCCESS){
		throw "Bad settings file - wrong debug level";
	}


	cameraParams.cacheSaveEnabled = true;
	TiXmlElement* pPtr = settings->FirstChildElement("cache");
	if(!pPtr){
		throw "Bad settings file - no cache setting for Camera";
	}
	if(pPtr->QueryBoolAttribute("saveEnabled", &cameraParams.cacheSaveEnabled) != TIXML_SUCCESS){
		cout << "Warning - no cacheSaveEnabled setting for Camera";
	}

	cameraParams.cacheLoadEnabled = true;
	if(pPtr->QueryBoolAttribute("loadEnabled", &cameraParams.cacheLoadEnabled) != TIXML_SUCCESS){
		cout << "Warning - no cacheLoadEnabled setting for Camera";
	}

	TiXmlElement* pInfer = settings->FirstChildElement("inference");
	if(!pInfer){
		throw "Bad settings file - no inference setting for Camera";
	}

	cameraParams.inferenceEnabled = false;
	if(pInfer->QueryBoolAttribute("enabled", &cameraParams.inferenceEnabled) != TIXML_SUCCESS){
		cout << "Warning - no inference enabled setting for Camera";
	}

	cameraParams.inferenceParams = readVectorSettings(pInfer, "params");
//	cout << "inference params = " << inferenceParams << endl;

	//TODO Add to xml settings file
	cameraParams.crossValidate = false;

	pPtr = settings->FirstChildElement("learning");
	if(!pPtr){
		throw "Bad settings file - no learning settings";
	}
	if(pPtr->QueryBoolAttribute("enabled", &cameraParams.learnEnabled) != TIXML_SUCCESS){
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
		cameraParams.learningDirs.push_back(string(pos + 1, dirTextEnd - pos - 1));
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
		if(cameraParams.labels.size() <= id){
			cameraParams.labels.resize(id + 1);
		}
		cameraParams.labels[id] = text;
		pLabel = pLabel->NextSiblingElement("label");
	}

	cameraParams.imuOrigRobot = readMatrixSettings(settings, "imu_position_robot", 4, 4);

	pPtr = settings->FirstChildElement("sensor");
	cameraParams.cameraOrigImu.resize(cameraParams.numCameras);
	cameraParams.cameraOrigLaser.resize(cameraParams.numCameras);
	cameraParams.cameraMatrix.resize(cameraParams.numCameras);
	cameraParams.distCoeffs.resize(cameraParams.numCameras);
	hierClassifiers.resize(cameraParams.numCameras);
	cameraParams.maskIgnore.resize(cameraParams.numCameras);
	for(int i = 0; i < cameraParams.numCameras; i++){
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

		cameraParams.cameraOrigImu[idx] = readMatrixSettings(pPtr, "imu_position_camera", 4, 4).t();
		cameraParams.cameraOrigLaser[idx] = readMatrixSettings(pPtr, "position_laser", 4, 4);
		cameraParams.cameraMatrix[idx] = readMatrixSettings(pPtr, "camera_matrix", 3, 3);
		cameraParams.distCoeffs[idx] = readMatrixSettings(pPtr, "dist_coeffs", 1, 5);
		cameraParams.maskIgnore[idx] = Mat(cameraParams.numRows, cameraParams.numCols, CV_32SC1, Scalar(0));

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
			selectPolygonPixels(poly, 1, cameraParams.maskIgnore[idx]);
			pPolygon = pPolygon->NextSiblingElement("polygon");
		}

		hierClassifiers[idx] = new HierClassifier(cameraParams.cameraMatrix[idx]);

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

void Camera::insertNewData(cv::Mat& dataAll, cv::Mat newData, int dataSkipped){
//	cout << "Camera::insertNewData" << endl;

//	cout << "dataAll.size() = " << dataAll.size() << endl;
//	cout << "newData.size() = " << newData.size() << endl;
//	cout << "dataSkipped = " << dataSkipped << endl;

	if(dataSkipped > dataAll.cols){
		dataSkipped = dataAll.cols;
	}

	Mat tmpDataAll(newData.rows, dataAll.cols - dataSkipped + newData.cols, newData.type());
//	cout << "tmpDataAll.size() = " << tmpDataAll.size() << endl;
//	cout << "dataSkipped = " << dataSkipped << endl;
	if(!dataAll.empty() && dataSkipped < dataAll.cols){
		dataAll.colRange(dataSkipped, dataAll.cols).
						copyTo(tmpDataAll.colRange(0, dataAll.cols - dataSkipped));
	}
	if(!newData.empty()){
		newData.copyTo(tmpDataAll.colRange(dataAll.cols - dataSkipped, tmpDataAll.cols));
	}
	dataAll = tmpDataAll;

//	cout << "End Camera::insertNewData" << endl;
}

/** \brief Assign label for each map segment relaying on pixel-wise labels
 *
 */
cv::Mat Camera::assignSegmentLabels(cv::Mat pixelLabels, cv::Mat coords){
	cout << "Camera::assignSegmentLabels" << endl;
	Mat segmentLabels(MAP_SIZE, MAP_SIZE, CV_32SC1, Scalar(-1));
	vector<Mat> segmentLabelsCount;
	for(int l = 0; l < cameraParams.labels.size(); ++l){
		segmentLabelsCount.push_back(Mat(MAP_SIZE, MAP_SIZE, CV_32SC1, Scalar(0)));
	}

//	cout << "counting" << endl;
//	cout << "coords.size() = " << coords.size() << endl;
	for(int d = 0; d < coords.cols; ++d){
		int xSegm = coords.at<float>(0, d)/MAP_RASTER_SIZE + MAP_SIZE/2;
		int ySegm = coords.at<float>(1, d)/MAP_RASTER_SIZE + MAP_SIZE/2;
//		cout << "(" << coords.at<float>(0, d) << ", " << coords.at<float>(1, d) << ")" << endl;

		int curLabel = pixelLabels.at<int>(d);
//		cout << "curLabel = " << curLabel << endl;
//		if(xSegm < 0 || xSegm >= MAP_SIZE || ySegm < 0 || ySegm >= MAP_SIZE || curLabel >= (int)segmentLabelsCount.size()){
//			cout << "curLabel = " << curLabel << endl;
//			cout << "(" << xSegm << ", " << ySegm << ")" << endl;
//		}
		if(xSegm >= 0 || xSegm < MAP_SIZE || ySegm >= 0 || ySegm < MAP_SIZE){
			if(curLabel >= 0){
				segmentLabelsCount[curLabel].at<int>(xSegm, ySegm)++;
			}
		}
	}

//	cout << "choosing best" << endl;
	for(int mapX = 0; mapX < MAP_SIZE; ++mapX){
		for(int mapY = 0; mapY < MAP_SIZE; ++mapY){
			int bestLabelScore = 0;
			int bestLabelInd = -1;
			for(int l = 0; l < segmentLabelsCount.size(); ++l){
				if(segmentLabelsCount[l].at<int>(mapX, mapY) > bestLabelScore){
					bestLabelInd = l;
					bestLabelScore = segmentLabelsCount[l].at<int>(mapX, mapY);
				}
			}
//			cout << "(" << mapX << ", " << mapY << ")" << endl;
//			cout << "bestLabelScore = " << bestLabelScore << endl;
//			cout << "bestLabelInd = " << bestLabelInd << endl;

			segmentLabels.at<int>(mapX, mapY) = bestLabelInd;
		}
	}
	cout << "End Camera::assignSegmentLabels" << endl;

	return segmentLabels;
}

void Camera::init3DVis(cv::viz::Viz3d& win){
	//window size
	win.setWindowSize(cv::Size(640, 480));
    //camera pose
    /// Let's assume camera has the following properties
	Vec3d camCoords(-4000.0f, 4000.0f, -4000.0f);
	Vec3d camFocalPoint(0.0f, 0.0f, 0.0f);
	Vec3d camYDir(-1.0f, 1.0f, 1.0f);
    Affine3f camPose = cv::viz::makeCameraPose(camCoords, camFocalPoint, camYDir);
    win.setViewerPose(camPose);
}

void Camera::draw3DVis(cv::viz::Viz3d& win,
					cv::Mat coords,
					cv::Mat colors,
					cv::Mat pos,
					cv::Mat segments,
					cv::Mat laserPointCloudOrigRobotMapCenter,
					cv::Mat segmentsManual,
					float goalDir,
					float bestDir,
					float bestDirRef,
					bool stopFlag)
{
	static const bool saveScreenshots = true;
	static int frameIdx = 0;

//    ///create a window
//    viz::Viz3d win("camera visualization");
	win.removeAllWidgets();

    ///add frame of reference
    Affine3f affineImuPos(cameraParams.imuOrigRobot);
    win.showWidget("frame of reference widget", viz::WCoordinateSystem(1000));

    //add grid
    viz::WGrid grid(Vec2i(MAP_SIZE, MAP_SIZE), Vec2d(MAP_RASTER_SIZE, MAP_RASTER_SIZE));
    win.showWidget("grid widget", grid);

    //robot pose
    viz::WCoordinateSystem robotFrameOfRef(500);
    Affine3f affinePos(pos);
    win.showWidget("robot frame of ref widget", robotFrameOfRef, affinePos);

//    win.setWidgetPose("robot frame of ref widget", affinePose);

    //pixel point cloud
    Mat coordsT = coords.t();
    Mat coordsVis = coordsT.reshape(4, 1);
//    cout << "coordsVis.size() = " << coordsVis.size() << endl;
//    cout << "colors.size() = " << colors.size() << endl;
    viz::WCloud pixelCloud(coordsVis, colors);
    pixelCloud.setRenderingProperty(viz::OPACITY, 0.75);
    win.showWidget("pixel cloud widget", pixelCloud);

    //laser point cloud
//    Mat coordsLaserT = laserPointCloudOrigRobotMapCenter.rowRange(0, 4).t();
//	Mat coordsLaserVis = coordsLaserT.reshape(4, 1);
////    cout << "coordsVis.size() = " << coordsVis.size() << endl;
////    cout << "colors.size() = " << colors.size() << endl;
//	viz::WCloud laserPointCloud(coordsLaserVis, viz::Color::yellow());
//	laserPointCloud.setRenderingProperty(viz::OPACITY, 0.5);
//	win.showWidget("laser point cloud widget", laserPointCloud);

    //results
    for(int mapX = 0; mapX < MAP_SIZE; ++mapX){
    	for(int mapY = 0; mapY < MAP_SIZE; ++mapY){
    		if(segments.at<int>(mapX, mapY) >= 0)
    		{
    			Point3d centerPt((mapX - MAP_SIZE/2) * MAP_RASTER_SIZE + MAP_RASTER_SIZE/2,
								(mapY - MAP_SIZE/2) * MAP_RASTER_SIZE + MAP_RASTER_SIZE/2,
								0);

    			viz::Color color;
    			if(segments.at<int>(mapX, mapY) == DRIVABLE_LABEL){
    				color = viz::Color::red();
    			}
    			else{
    				color = viz::Color::green();
    			}

    			viz::WPlane segPlane(centerPt,
    								Vec3d(0.0, 0.0, 1.0) /*normal*/,
									Vec3d(0.0, 1.0, 0.0) /*new y axis*/,
									Size2d(MAP_RASTER_SIZE, MAP_RASTER_SIZE) /*size*/,
									color);
    			segPlane.setRenderingProperty(viz::OPACITY, 0.20);

    			int segId = mapX * MAP_SIZE + mapY;
    			char buf[10];
    			sprintf(buf, "%d", segId);
    			win.showWidget(String("plane widget") + String(buf), segPlane);
    		}
    	}
    }

    //manual labels
    if(!segmentsManual.empty()){
		for(int mapX = 0; mapX < MAP_SIZE; ++mapX){
			for(int mapY = 0; mapY < MAP_SIZE; ++mapY){
				if(segmentsManual.at<int>(mapX, mapY) < 0 &&
					segments.at<int>(mapX, mapY) >= 0)
				{
					Point3d centerPt((mapX - MAP_SIZE/2) * MAP_RASTER_SIZE + MAP_RASTER_SIZE/2,
									(mapY - MAP_SIZE/2) * MAP_RASTER_SIZE + MAP_RASTER_SIZE/2,
									0);
					viz::WPlane segPlane(centerPt,
										Vec3d(0.0, 0.0, 1.0) /*normal*/,
										Vec3d(0.0, 1.0, 0.0) /*new y axis*/,
										Size2d(MAP_RASTER_SIZE, MAP_RASTER_SIZE) /*size*/,
										viz::Color::blue());
					segPlane.setRenderingProperty(viz::OPACITY, 0.5);

					int segId = mapX * MAP_SIZE + mapY;
					char buf[10];
					sprintf(buf, "%d", segId);
					win.showWidget(String("manual plane widget") + String(buf), segPlane);
				}
			}
		}
    }


	static const float arrowLen = 1000;
	Mat posNoOrient = pos.clone();
	Mat eye33 = Mat::eye(3, 3, CV_32FC1);
	eye33.copyTo(Mat(posNoOrient, Rect(0, 0, 3, 3)));
	Affine3f affinePosNoOrient(posNoOrient);

    viz::WArrow bestDirArrow = viz::WArrow(Point3d(0, 0, -100),
										Point3d(arrowLen*cos(bestDir*PI/180), arrowLen*sin(bestDir*PI/180), -100),
										0.03,
										viz::Color::yellow());
    win.showWidget(String("best dir arrow"), bestDirArrow, affinePosNoOrient);

    viz::WArrow bestDirRefArrow = viz::WArrow(Point3d(0, 0, -100),
										Point3d(arrowLen*cos(bestDirRef*PI/180), arrowLen*sin(bestDirRef*PI/180), -100),
										0.03,
										viz::Color::green());
    win.showWidget(String("best dir ref arrow"), bestDirRefArrow, affinePosNoOrient);

    viz::WArrow goalDirArrow = viz::WArrow(Point3d(0, 0, -100),
										Point3d(arrowLen*cos(goalDir*PI/180), arrowLen*sin(goalDir*PI/180), -100),
										0.03,
										viz::Color::magenta());
    win.showWidget(String("goal dir arrow"), goalDirArrow, affinePosNoOrient);

    // Event loop is over when pressed q, Q, e, E
	// Start event loop once for 5 + 5 millisecond
    int count = 0;
    win.spinOnce(5, true);
	while(!win.wasStopped() && count < 20)
	{
		// Interact with window

		// Event loop for 5 + 5 millisecond
		win.spinOnce(5, true);
		waitKey(5);

		if(!stopFlag){
			count++;
		}
	}
    if(saveScreenshots){
    	cout << "Saving screenshot" << endl;
    	char buf[20];
    	sprintf(buf, "viz/viz%04d.png", frameIdx++);
    	cout << "Filename: " << buf << endl;
    	win.saveScreenshot(buf);
    	cout << "End saving screenshot" << endl;
    }
}

void Camera::updatePixelData(cv::Mat& pixelCoordsAll,
							std::vector<cv::Mat>& classResultsAll,
							cv::Mat& manualLabelsAll,
							cv::Mat& pixelColorsAll,
							std::queue<ClassResult>& classResultsHist,
							const cv::Mat mapMove,
							std::chrono::high_resolution_clock::time_point timestamp,
							const cv::Mat pixelCoords,
							const std::vector<cv::Mat>& classResults,
							const cv::Mat image,
							const cv::Mat manualLabelsOnImage)
{
	//merge with previous classification results

	//remove old pixels
	int pixelsSkipped = 0;

	//if new series of data begins - eg. new dir
	bool removeAllFlag = false;

//	cout << "removing old" << endl;
	if(!classResultsHist.empty()){
		if(timestamp < classResultsHist.front().timestamp){
			pixelsSkipped = pixelCoordsAll.cols;
			while(!classResultsHist.empty()){
				classResultsHist.pop();
			}
			removeAllFlag = true;
		}
		while(!removeAllFlag &&
				(timestamp - classResultsHist.front().timestamp) > std::chrono::milliseconds(cameraParams.pixelsTimeout))
		{
			pixelsSkipped += classResultsHist.front().numPixels;
			classResultsHist.pop();
			if(classResultsHist.empty()){
				break;
			}
		}
	}

	//move old pixels to new map ommiting those removed
	if(pixelsSkipped < pixelCoordsAll.cols){
		Mat tmpPixelCoordsAll = mapMove * pixelCoordsAll.colRange(pixelsSkipped, pixelCoordsAll.cols);
		tmpPixelCoordsAll.copyTo(pixelCoordsAll.colRange(pixelsSkipped, pixelCoordsAll.cols));
	}

	//add new
//	cout << "adding new" << endl;
	//coords
	insertNewData(pixelCoordsAll, pixelCoords, pixelsSkipped);

	//classification results
	for(int l = 0; l < cameraParams.labels.size(); ++l){
		insertNewData(classResultsAll[l], classResults[l].reshape(0, 1), pixelsSkipped);
	}

	//manual labels
	if(!manualLabelsOnImage.empty()){
		insertNewData(manualLabelsAll, manualLabelsOnImage.reshape(0, 1), pixelsSkipped);
	}

	insertNewData(pixelColorsAll, image.reshape(0, 1), pixelsSkipped);

//		cout << "pixelCoordsAll.size() = " << pixelCoordsAll.size() << endl;
//		cout << "pixelColorsAll.size() = " << pixelColorsAll.size() << endl;

	classResultsHist.push(ClassResult(timestamp, pixelCoords.cols));
}

void Camera::prepareSegmentInfo(std::vector<cv::Mat>& segmentPriors,
								std::vector<cv::Mat>& segmentFeats,
								std::vector<int>& segmentPixelCount,
								cv::Mat pixelCoords,
								cv::Mat pixelColors,
								const std::vector<cv::Mat>& classResults,
								cv::Mat pointCloudOrigRobotMapCenter,
								cv::Mat segmentMask)
{
	segmentPriors.clear();
	segmentFeats.clear();
	segmentPixelCount.clear();

	std::vector<int> segmentPointCount;

    for(int mapX = 0; mapX < MAP_SIZE; ++mapX){
    	for(int mapY = 0; mapY < MAP_SIZE; ++mapY){
    		//prior for each label
    		segmentPriors.push_back(Mat(cameraParams.labels.size(), 1, CV_32FC1, Scalar(0.0)));
    		//5 features - mean R (camera), G (camera), B (camera), dist (laser), intensity (laser)
    		segmentFeats.push_back(Mat(5, 1, CV_32FC1, Scalar(0.0)));
    		segmentPixelCount.push_back(0);

    		segmentPointCount.push_back(0);
    	}
    }
//    cout << "segmentPriors.size() = " << segmentPriors.size() << endl;
//    cout << "pixels" << endl;
	for(int d = 0; d < pixelCoords.cols; ++d){
		bool skipPixel = false;
		for(int l = 0; l < cameraParams.labels.size(); ++l){
			if(classResults[l].at<float>(d) < 0.0){
				skipPixel = true;
				break;
			}
		}

		if(!skipPixel){
			int xSegm = pixelCoords.at<float>(0, d)/MAP_RASTER_SIZE + MAP_SIZE/2;
			int ySegm = pixelCoords.at<float>(1, d)/MAP_RASTER_SIZE + MAP_SIZE/2;

			if(xSegm >= 0 && xSegm < MAP_SIZE && ySegm >= 0 && ySegm < MAP_SIZE){
				int segId = xSegm*MAP_SIZE + ySegm;

	//			cout << "segId = " << segId << endl;
				++segmentPixelCount[segId];
	//			cout << "segmentPixelCount[segId] = " << segmentPixelCount[segId] << endl;
				for(int l = 0; l < cameraParams.labels.size(); ++l){
					segmentPriors[segId].at<float>(l) += classResults[l].at<float>(d);
				}
				for(int col = 0; col < 3; ++col){
					segmentFeats[segId].at<float>(col) += pixelColors.at<Vec3b>(d)[col];
				}
			}
		}
	}
//	cout << "points" << endl;
	for(int d = 0; d < pointCloudOrigRobotMapCenter.cols; ++d){
		int xSegm = pointCloudOrigRobotMapCenter.at<float>(0, d)/MAP_RASTER_SIZE + MAP_SIZE/2;
		int ySegm = pointCloudOrigRobotMapCenter.at<float>(1, d)/MAP_RASTER_SIZE + MAP_SIZE/2;

		if(xSegm >= 0 && xSegm < MAP_SIZE && ySegm >= 0 && ySegm < MAP_SIZE){
			int segId = xSegm*MAP_SIZE + ySegm;

//			cout << "segId = " << segId << endl;
			++segmentPointCount[segId];
//			cout << "segmentPixelCount[segId] = " << segmentPixelCount[segId] << endl;

//			cout << "pointCloudOrigRobotMapCenter.at<float>(4, d) = " << pointCloudOrigRobotMapCenter.at<float>(4, d) << endl;
//			cout << "pointCloudOrigRobotMapCenter.at<float>(5, d) = " << pointCloudOrigRobotMapCenter.at<float>(5, d) << endl;
			segmentFeats[segId].at<float>(3) += pointCloudOrigRobotMapCenter.at<float>(4, d);
			segmentFeats[segId].at<float>(4) += pointCloudOrigRobotMapCenter.at<float>(5, d);
		}
	}

	for(int seg = 0; seg < segmentPriors.size(); ++seg){
		bool segExclude = false;
		if(!segmentMask.empty()){
			int curX = seg / MAP_SIZE;
			int curY = seg % MAP_SIZE;
			if(segmentMask.at<unsigned char>(curX, curY) == 0){
				segExclude = true;
			}
		}
		if(segmentPixelCount[seg] > cameraParams.entryWeightThreshold &&
			!segExclude)
		{
//			cout << "seg = " << seg << endl;
			segmentPriors[seg] /= segmentPixelCount[seg];
			//ensure that no prior has value 0.0 or 1.0 - unacceptable during inference
			static const float priorBias = 0.05;
			segmentPriors[seg] = segmentPriors[seg] * (1.0 - priorBias) + (1.0 / cameraParams.labels.size()) * priorBias;

			segmentFeats[seg].at<float>(0) /= segmentPixelCount[seg];
			segmentFeats[seg].at<float>(1) /= segmentPixelCount[seg];
			segmentFeats[seg].at<float>(2) /= segmentPixelCount[seg];
			//if any point of the point cloud falls into segment
			if(segmentPointCount[seg] > 0){
				segmentFeats[seg].at<float>(3) /= segmentPointCount[seg];
				segmentFeats[seg].at<float>(4) /= segmentPointCount[seg];
			}
			else{
				segmentFeats[seg].at<float>(3) = 0.0;
				segmentFeats[seg].at<float>(4) = 0.0;
			}

//			cout << "segmentFeats[" << seg << "] = " << segmentFeats[seg] << endl;
		}
		else{
			segmentPixelCount[seg] = 0;
		}
	}
}

void addEdgeToPgm(Cluster* a,
					Cluster* b,
					vector<RandVar*> sepset /* id sorted */)
{
//	cout << "adding edge from " << a->id() << " to " << b->id() << endl;
	vector<Cluster*> aNh = a->nh();
	auto it = upper_bound(aNh.begin(), aNh.end(), b, compIdClust);
	int pos = it - aNh.begin();
//	cout << "inseting b at pos " << pos << ", aNh.size() = " << aNh.size() << endl;
	aNh.insert(it, b);
	a->setNh(aNh);

	vector<vector<RandVar*> > aSepsets = a->sepsets();
//	cout << "inseting sepset at pos " << pos << ", aSepsets.size() = " << aSepsets.size() << endl;
	aSepsets.insert(aSepsets.begin() + pos, sepset);
	a->setSepsets(aSepsets);

	vector<Cluster*> bNh = b->nh();
	it = upper_bound(bNh.begin(), bNh.end(), a, compIdClust);
	pos = it - bNh.begin();
//	cout << "inseting a at pos " << pos << ", bNh.size() = " << bNh.size() << endl;
	bNh.insert(it, a);
//	cout << "end inserting" << endl;
	b->setNh(bNh);

//	cout << "sep" << endl;
	vector<vector<RandVar*> > bSepsets = b->sepsets();
//	cout << "inseting sepset at pos " << pos << ", bSepsets.size() = " << bSepsets.size() << endl;
	bSepsets.insert(bSepsets.begin() + pos, sepset);
	b->setSepsets(bSepsets);
}

void Camera::constructPgm(Pgm& pgm,
						std::map<int, int>& segIdToVarClusterId,
						std::map<int, int>& segIdToRandVarId,
						std::vector<double>& obsVec,
						const std::vector<cv::Mat>& segmentPriors,
						const std::vector<cv::Mat>& segmentFeats,
						const std::vector<int>& segmentPixelCount)
{
	vector<RandVar*> randVars;
	vector<Cluster*> clusters;
	vector<Feature*> features;
	vector<double> params;

	int numSegFeat = segmentFeats.front().rows;
	int numLabels = segmentPriors.front().rows;

	//observation vector
	for(int seg = 0; seg < segmentPixelCount.size(); ++seg){
		if(segmentPixelCount[seg] > 0){
			for(int l = 0; l < numLabels; ++l){
				obsVec.push_back(segmentPriors[seg].at<float>(l));
			}
			for(int f = 0; f < numSegFeat; ++f){
				obsVec.push_back(segmentFeats[seg].at<float>(f));
			}
		}
	}

	//random variables
	int nextRandVarId = 0;
	vector<double> randVarVals;
	for(int l = 0; l < cameraParams.labels.size(); ++l){
		randVarVals.push_back(l);
	}
	for(int seg = 0; seg < segmentPixelCount.size(); ++seg){
		if(segmentPixelCount[seg] > 0){
			randVars.push_back(new RandVar(nextRandVarId, randVarVals));
			segIdToRandVarId[seg] = nextRandVarId;
			++nextRandVarId;
		}
	}

	//features
	int nextFeatId = 0;
	vector<int> nodeFeatObsNums;
	for(int l = 0; l < numLabels; ++l){
		nodeFeatObsNums.push_back(l);
	}
	features.push_back(new TerClassNodeFeature(nextFeatId, nextFeatId, nodeFeatObsNums));
	++nextFeatId;

	for(int f = 0; f < numSegFeat; ++f){
		features.push_back(new TerClassPairFeature(nextFeatId, nextFeatId, vector<int>{f, f + numSegFeat}));
		++nextFeatId;
	}

	params = vector<double>(nextFeatId, 1.0);

	//var clusters
	int nextClusterId = 0;
	for(int seg = 0; seg < segmentPixelCount.size(); ++seg){
		if(segmentPixelCount[seg] > 0){
			int randVarId = segIdToRandVarId[seg];
			Cluster* curCluster = new Cluster(nextClusterId,
												vector<Feature*>{},
												vector<RandVar*>{randVars[randVarId]},
												vector<int>{0});
			segIdToVarClusterId[seg] = nextClusterId;
			clusters.push_back(curCluster);
			++nextClusterId;
		}
	}

	//node factor clusters
	map<int, int> segIdToNodeFactClusterId;
	for(int seg = 0; seg < segmentPixelCount.size(); ++seg){
		if(segmentPixelCount[seg] > 0){
			int randVarId = segIdToRandVarId[seg];
			vector<int> obsVecIdxs;
			for(int l = 0; l < numLabels; ++l){
				obsVecIdxs.push_back(randVarId * (numLabels + numSegFeat) + l);
			}
			Cluster* curCluster = new Cluster(nextClusterId,
												vector<Feature*>(features.begin(), features.begin() + 1),
												vector<RandVar*>{randVars[randVarId]},
												vector<int>{0},
												obsVecIdxs);
			segIdToNodeFactClusterId[seg] = nextClusterId;
			clusters.push_back(curCluster);

			//add edge to cluster for variable
			addEdgeToPgm(curCluster, clusters[segIdToVarClusterId[seg]], vector<RandVar*>{randVars[randVarId]});

			++nextClusterId;
		}
	}

	//pair-wise factor clusters
	vector<set<int> > addedEdges(segmentPixelCount.size());
	for(int seg = 0; seg < segmentPixelCount.size(); ++seg){
		if(segmentPixelCount[seg] > 0){
//			cout << "seg = " << seg << endl;
			int randVarId = segIdToRandVarId[seg];

			//Neighborhood
			int nhood[][2] = {{1, 0},
								{0, 1},
								{-1, 0},
								{0, -1}};
			int curX = seg / MAP_SIZE;
			int curY = seg % MAP_SIZE;
			for(int nh = 0; nh < (int)(sizeof(nhood)/sizeof(nhood[0])); ++nh){
				int nX = curX + nhood[nh][0];
				int nY = curY + nhood[nh][1];
				if(nX >= 0 && nX < MAP_SIZE && nY >= 0 && nY < MAP_SIZE){
					int nId = nX*MAP_SIZE + nY;
//					cout << "nId = " << nId << endl;
//					cout << "segIdToVarClusterId.count(nId) = " << segIdToVarClusterId.count(nId) << endl;
//					cout << "addedEdges[seg].count(nId) = " << addedEdges[seg].count(nId) << endl;
					//if neighboring segment exists and edge not added
					if(segIdToVarClusterId.count(nId) > 0 &&
						addedEdges[seg].count(nId) == 0)
					{
//						cout << "Adding edge between " << seg << " and " << nId << endl;
						int nRandVarId = segIdToRandVarId[nId];
						vector<int> obsVecIdxs;
						for(int f = 0; f < numSegFeat; ++f){
							obsVecIdxs.push_back(randVarId * (numLabels + numSegFeat) + numLabels + f);
						}
						for(int f = 0; f < numSegFeat; ++f){
							obsVecIdxs.push_back(nRandVarId * (numLabels + numSegFeat) + numLabels + f);
						}

						vector<RandVar*> clustRandVars;
						if(randVarId < nRandVarId){
							clustRandVars.push_back(randVars[randVarId]);
							clustRandVars.push_back(randVars[nRandVarId]);
						}
						else{
							clustRandVars.push_back(randVars[nRandVarId]);
							clustRandVars.push_back(randVars[randVarId]);
						}

						Cluster* curCluster = new Cluster(nextClusterId,
															vector<Feature*>(features.begin() + 1, features.end()),
															clustRandVars,
															vector<int>{0, 1},
															obsVecIdxs);
						clusters.push_back(curCluster);

						//add edge to cluster for variable
						addEdgeToPgm(curCluster, clusters[segIdToVarClusterId[seg]], vector<RandVar*>{randVars[randVarId]});
						addEdgeToPgm(curCluster, clusters[segIdToVarClusterId[nId]], vector<RandVar*>{randVars[nRandVarId]});

						addedEdges[seg].insert(nId);
						addedEdges[nId].insert(seg);

						++nextClusterId;
					}
				}
			}
		}
	}


	pgm = Pgm(randVars, clusters, features);
	pgm.params() = params;

//	cout << "nextRandVarId = " << nextRandVarId << endl;
//	cout << "nextFeatId = " << nextFeatId << endl;
//	cout << "nextClusterId = " << nextClusterId << endl;
//	cout << "obsVec = " << obsVec << endl;
}

cv::Mat Camera::inferTerrainLabels(const Pgm& pgm,
									const std::vector<double>& obsVec,
									const std::map<int, int>& mapSegIdToVarClusterId)
{
	vector<vector<double> > marg;
	vector<vector<vector<double> > > msgs;
	vector<vector<double> > retVals;
//	vector<double> params{1.65718, -0.919813, -0.843494, -0.793375};

	bool calibrated = Inference::compMAPParam(pgm,
											marg,
											msgs,
											cameraParams.inferenceParams,
											obsVec);
//	cout << "calibrated = " << calibrated << endl;

    for(int mapX = 0; mapX < MAP_SIZE; ++mapX){
    	for(int mapY = 0; mapY < MAP_SIZE; ++mapY){
    		int segId = mapX * MAP_SIZE + mapY;
    		//if segment labeling is inferred
    		if(mapSegIdToVarClusterId.count(segId) > 0){
    			int varClusterId = mapSegIdToVarClusterId.at(segId);
//    			cout << "marg for seg " << segId << " = " << marg[varClusterId] << endl;
    		}
    	}
    }

	retVals = Inference::decodeMAP(pgm,
									marg,
									msgs,
									cameraParams.inferenceParams,
									obsVec);

	Mat ret(MAP_SIZE, MAP_SIZE, CV_32SC1, Scalar(-1));
    for(int mapX = 0; mapX < MAP_SIZE; ++mapX){
    	for(int mapY = 0; mapY < MAP_SIZE; ++mapY){
    		int segId = mapX * MAP_SIZE + mapY;
    		//if segment labeling is inferred
    		if(mapSegIdToVarClusterId.count(segId) > 0){
    			int varClusterId = mapSegIdToVarClusterId.at(segId);
    			const vector<double>& varVals = retVals[varClusterId];
    			//only 1 variable representing current terrain segment
    			int curLab = Pgm::roundToInt(varVals.front());
    			ret.at<int>(mapX, mapY) = curLab;
    		}
    	}
    }

	return ret;
}

void Camera::computeBestDirLocalMap(cv::Mat segmentResults,
									cv::Mat posOrigMapCenter,
									cv::Mat mapCenterOrigGlobal,
									float goalDirGlobal,
									float& bestDirLocalMap,
									float& goalDirLocalMap)
{
	LocalPlanner::Parameters localPlannerParams;
	localPlannerParams.debug = 0;
	localPlannerParams.avoidObstacles = 1;
	localPlannerParams.histResolution = 10;
	localPlannerParams.gauss3sig = 40;
	localPlannerParams.maxDistance = 3000;
	localPlannerParams.backwardsPenalty = 1.5;

	Mat segmentResultsFloat(segmentResults.size(), CV_32FC1, Scalar(0.0));
	for (int x = 0; x < MAP_SIZE; x++) {
		for (int y = 0; y < MAP_SIZE; y++) {
			if(segmentResults.at<int>(x, y) >= 0 &&
				segmentResults.at<int>(x, y) != DRIVABLE_LABEL)
			{
				segmentResultsFloat.at<float>(x, y) = 1.0;
			}
		}
	}

	int numSectors = (float)360 / localPlannerParams.histResolution + 0.5;
	vector<float> histSectors(numSectors, 0.0);
	LocalPlanner::updateHistogram(histSectors,
								posOrigMapCenter,
								segmentResultsFloat,
								localPlannerParams);

	if(cameraParams.debugLevel >= 1){
//				cout << "histSectors = " << histSectors << endl;
		cout << "smoothing" << endl;
	}

	LocalPlanner::smoothHistogram(histSectors, localPlannerParams);

	if(cameraParams.debugLevel >= 1){
//				cout << "histSectors = " << histSectors << endl;
		cout << "determining goal in local map" << endl;

//				cout << "mapCentersOrigGlobal.size() = " << mapCentersOrigGlobal.size() <<
//						", goalDirsGlobal.size() = " << goalDirsGlobal.size() << endl;
	}

	goalDirLocalMap = LocalPlanner::determineGoalInLocalMap(mapCenterOrigGlobal, goalDirGlobal);

	if(cameraParams.debugLevel >= 1){
		cout << "finding optim sector" << endl;
	}

	/// optimal direction in the local map - nearest to the goal
	bestDirLocalMap = LocalPlanner::findOptimSector(histSectors,
													posOrigMapCenter,
													goalDirLocalMap,
													localPlannerParams);
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

std::vector<double> Camera::readVectorSettings(TiXmlElement* parent, const char* node){
	TiXmlElement* ptr = parent->FirstChildElement(node);
	if(!ptr){
		throw (string("Bad settings file - no ") + string(node)).c_str();
	}
	//cout << "node: " << node << ", value: " << ptr->GetText() << endl;
	stringstream tmpStr(ptr->GetText());
	vector<double> ret;
	while(!tmpStr.fail()){
		float tmpVal;
		tmpStr >> tmpVal;
		if(tmpStr.fail()){
			break;
		}
		ret.push_back(tmpVal);
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
	for(int c = 0; c < cameraParams.numCameras; c++){
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
		//TODOa correct using TiXmlText
		pEntry->SetValue(tmpStr.str());
	}
	doc.SaveFile(cacheFile.c_str());*/
	for(int c = 0; c < cameraParams.numCameras; c++){
		char buffer[10];
		sprintf(buffer, "%02d.xml", c);
		hierClassifiers[c]->saveCache(cacheFile.string() + buffer);
	}
}

//Inserts computed constraints into map
void Camera::insertConstraints(	cv::Mat map,
								std::chrono::high_resolution_clock::time_point curTimestampMap,
								cv::Mat mapMove)
{
//	std::unique_lock<std::mutex> lck(mtxConstr);
//	if(!constraints.empty() && curTimestampMap < curTimestamp){
//		for(int x = 0; x < MAP_SIZE; x++){
//			for(int y = 0; y < MAP_SIZE; y++){
//				map.at<float>(x, y) = max(map.at<float>(x, y), constraints.at<float>(x, y));
//				//cout << "constraints.at<float>(x, y) = " << constraints.at<float>(x, y) << endl;
//				//cout << "map.at<float>(x, y) = " << map.at<float>(x, y) << endl;
//			}
//		}
//	}
//	lck.unlock();


	if(!pixelCoordsMapOrigRobotMapCenter.empty()){
		std::chrono::high_resolution_clock::time_point timeBegin;
		std::chrono::high_resolution_clock::time_point timeEndMoving;
		std::chrono::high_resolution_clock::time_point timeEndPreparing;
		std::chrono::high_resolution_clock::time_point timeEndConstructing;
		std::chrono::high_resolution_clock::time_point timeEndInfering;

		timeBegin = std::chrono::high_resolution_clock::now();

		std::unique_lock<std::mutex> lckClassRes(mtxClassResults);

		timestampMap = curTimestampMap;

//		cout << "moving map" << endl;
		//move map
//		cout << "pixelCoordsMapOrigRobotMapCenter.size() = " << pixelCoordsMapOrigRobotMapCenter.size() << endl;
//		cout << "mapMove.size() = " << mapMove.size() << endl;
		pixelCoordsMapOrigRobotMapCenter = mapMove * pixelCoordsMapOrigRobotMapCenter;

		mapMoveSinceGetPointCloud = mapMove * mapMoveSinceGetPointCloud;

		Mat pixelCoordsCopyOrigRobotMapCenter = pixelCoordsMapOrigRobotMapCenter.clone();
		Mat pixelColorsCopy = pixelColorsMap.clone();
		vector<Mat> classResultsCopy;
		for(int l = 0; l < classResultsMap.size(); ++l){
			classResultsCopy.push_back(classResultsMap[l].clone());
		}
		Mat pointCloudCopyOrigRobotMapCenter = pointCloudMapOrigRobotMapCenter.clone();

		lckClassRes.unlock();

		timeEndMoving = std::chrono::high_resolution_clock::now();
		//run inference
//		cout << "running inference" << endl;
		Mat segmentResults;

		if(cameraParams.inferenceEnabled){
			std::vector<cv::Mat> segmentPriors;
			std::vector<cv::Mat> segmentFeats;
			std::vector<int> segmentPixelCount;
			Pgm pgm;
			std::map<int, int> segIdToVarClusterId;
			std::map<int, int> segIdToRandVarId;
			std::vector<double> obsVec;
		//		Mat segmentMask(segmentManualLabels.size(), CV_32SC1, Scalar(0));
		//		segmentMask.setTo(Scalar(1), segmentManualLabels >= 0);

	//		cout << "prepare segment info" << endl;
			prepareSegmentInfo(segmentPriors,
								segmentFeats,
								segmentPixelCount,
								pixelCoordsCopyOrigRobotMapCenter,
								pixelColorsCopy,
								classResultsCopy,
								pointCloudCopyOrigRobotMapCenter);


			timeEndPreparing = std::chrono::high_resolution_clock::now();
	//		cout << "construct pgm" << endl;
			constructPgm(pgm,
						segIdToVarClusterId,
						segIdToRandVarId,
						obsVec,
						segmentPriors,
						segmentFeats,
						segmentPixelCount);

			timeEndConstructing = std::chrono::high_resolution_clock::now();
	//		cout << "infer terrain labels" << endl;
			segmentResults = inferTerrainLabels(pgm,
												obsVec,
												segIdToVarClusterId);

			pgm.deleteContents();
			for(int f = 0; f < pgm.feats().size(); ++f){
				delete pgm.feats()[f];
			}
		}
		else{
			Mat inferResults(1, pixelCoordsCopyOrigRobotMapCenter.cols, CV_32FC1);

			for(int d = 0; d < pixelCoordsCopyOrigRobotMapCenter.cols; ++d){
				int bestLabelInd = -1;
				float bestLabelScore = 0;
				for(int l = 0; l < classResultsCopy.size(); ++l){
					if(classResultsCopy[l].at<float>(0, d) > bestLabelScore){
						bestLabelInd = l;
						bestLabelScore = classResultsCopy[l].at<float>(0, d);
					}
				}
				inferResults.at<int>(d) = bestLabelInd;
			}

			//assign result for each observed segment - temporary, should be obtained during inference
			cout << "assigning results" << endl;
			segmentResults = assignSegmentLabels(inferResults, pixelCoordsCopyOrigRobotMapCenter);
	//		cout << "segmentResults = " << segmentResults << endl;
		}

	//	cout << "infer results" << endl;
	//	Mat inferResults(1, pixelCoordsMap.cols, CV_32SC1);
	//	for(int d = 0; d < pixelCoordsMap.cols; ++d){
	//		int xSegm = pixelCoordsMap.at<float>(0, d)/MAP_RASTER_SIZE + MAP_SIZE/2;
	//		int ySegm = pixelCoordsMap.at<float>(1, d)/MAP_RASTER_SIZE + MAP_SIZE/2;
	//
	//		inferResults.at<int>(d) = segmentResults.at<int>(xSegm, ySegm);
	//	}

		//copy results to map
		for(int x = 0; x < MAP_SIZE; ++x){
			for(int y = 0; y < MAP_SIZE; ++y){
				float curConstr = 0.0;
				if(segmentResults.at<int>(x, y) >= 0 &&
					segmentResults.at<int>(x, y) != DRIVABLE_LABEL)
				{
					curConstr = 1.0;
				}
				map.at<float>(x, y) = max(map.at<float>(x, y), curConstr);
			}
		}

		timeEndInfering = std::chrono::high_resolution_clock::now();
		cout << "Moving time: " << std::chrono::duration_cast<std::chrono::milliseconds>(timeEndMoving - timeBegin).count() << " ms" << endl;
		cout << "Preparing time: " << std::chrono::duration_cast<std::chrono::milliseconds>(timeEndPreparing - timeEndMoving).count() << " ms" << endl;
		cout << "Constructing time: " << std::chrono::duration_cast<std::chrono::milliseconds>(timeEndConstructing - timeEndPreparing).count() << " ms" << endl;
		cout << "Infering time: " << std::chrono::duration_cast<std::chrono::milliseconds>(timeEndInfering - timeEndPreparing).count() << " ms" << endl;

	}
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
		if(!tmp.empty() && (tmp.rows != cameraParams.numRows || tmp.cols != cameraParams.numCols)){
			Mat tmpResized;
			resize(tmp, tmpResized, Size(cameraParams.numCols, cameraParams.numRows));
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
		for(int l = 0; l < cameraParams.labels.size(); l++){
			coloredOriginal.setTo(colors[l], sharedClassifiedImage == l);
		}
		ret = coloredOriginal * 0.25 + sharedOriginalImage * 0.75;
		ret.setTo(Scalar(0, 0, 0), cameraParams.maskIgnore.front() != 0);
	}
	lck.unlock();

	return ret;
}

void Camera::getPixelPointCloud(cv::Mat& pixelCoords,
								cv::Mat& pixelColors)
{
	std::unique_lock<std::mutex> lck(mtxClassResults);

	pixelCoords = pixelCoordsMapOrigRobotMapCenter.clone();
	pixelColors = pixelColorsMap.clone();

	lck.unlock();
}

void Camera::open(std::vector<std::string> device){
	std::unique_lock<std::mutex> lck(mtxDevice);
	for(int i = 0; i < min((int)device.size(), cameraParams.numCameras); i++){
		cout << "Opening device: " << device[i] << endl;
		cameras[i].open(0);
		if(!cameras[i].isOpened()){
			throw "Cannot open camera device";
		}
	}
	lck.unlock();
}

void Camera::close(){
	cout << "Closing cameras" << endl;

	std::unique_lock<std::mutex> lck(mtxDevice);
//	cout << "cameras.size() = " << cameras.size() << endl;
	for(int i = 0; i < cameras.size(); i++){
//		cout << "i = " << i << endl;
		if(cameras[i].isOpened()){
//			cout << "Releasing" << endl;
			cameras[i].release();
		}
	}
	lck.unlock();
	cout << "End closing cameras" << endl;
}

bool Camera::isOpen(){
	return cameras.front().isOpened();
}
