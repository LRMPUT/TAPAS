/*
 * Camera.cpp
 *
 *  Created on: 08-07-2013
 *      Author: jachu
 */

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
//STL
#include <cmath>
#include <sstream>
#include <algorithm>
#include <fstream>
#include <sstream>
//RobotsIntellect
#include "Camera.h"

using namespace boost;
using namespace std;

/*#define CAMERA_Z 1
#define CAMERA_X_ANGLE 45
#define CAMERA_Y_ANGLE 45
#define CAMERAS_COUNT 2
#define ROWS 480
#define COLS 640*/

#define PI 3.14159265359

#define POLY_VERT 4
#define X_STEP 100
#define Y_STEP 100
#define X_RES 50
#define Y_RES 50
#define PLANE_Z -100
#define DRIVABLE_LABEL 3
#define LEFT_CAMERA 0
#define RIGHT_CAMERA 1

#define CHANNELS_USED 2
#define SAMPLE_PACK 1500

#define NO_CUDA

using namespace cv;
using namespace gpu;
using namespace std;


Camera::Camera(MovementConstraints* imovementConstraints, TiXmlElement* settings) :
		movementConstraints(imovementConstraints)
{
	if(!settings){
		throw "Bad settings file - entry Camera not found";
	}
	readSettings(settings);

	computeImagePolygons();

#ifndef NO_CUDA
	cout << "Available CUDA devices: " << getCudaEnabledDeviceCount() << endl;
	setDevice(0);
	DeviceInfo gpuInfo;
	cout << "Version: " << gpuInfo.majorVersion() << "." << gpuInfo.minorVersion() << endl;
	cout << "Number of processors: " << gpuInfo.multiProcessorCount() << endl;
#endif //NO_CUDA
}

Camera::~Camera(){
	for(int i = 0; i < hierClassifiers.size(); i++){
		delete hierClassifiers[i];
	}
}

void Camera::computeConstraints(std::vector<cv::Mat> image){
	cout << "Computing constraints" << endl;
	constraints = Mat(X_RES, Y_RES, CV_32FC1, Scalar(0));
	Mat constrNorm(X_RES, Y_RES, CV_32FC1, Scalar(0));
	namedWindow("original");
	namedWindow("prob asphalt");
	namedWindow("constraints");
	for(int c = 0; c < image.size(); c++){
		vector<Mat> classRes = hierClassifiers[c]->classify(image[c]);
		//cout << "Drawing polygons" << endl;
		Mat regions(image[c].rows, image[c].cols, CV_32SC1, Scalar(-1));
		for(int xInd = 0; xInd < X_RES; xInd++){
			for(int yInd = 0; yInd < Y_RES; yInd++){
				selectPolygonPixels(imagePolygons[c][xInd][yInd],
									xInd*X_RES + yInd,
									regions);
				/*float probDriv = 0;
				for(int l = 0; l < classRes.size(); l++){
					if(l == DRIVABLE_LABEL){
						Mat tmp(classRes[l].rows, classRes[l].rows, CV_32FC1, Scalar(0));
						classRes[l].copyTo(tmp, mask);
						probDriv += sum(tmp)[0];
					}
				}
				constraints.at<float>(xInd, yInd) = probDriv;*/
			}
		}
		//cout << "adding probabilities, classRes.size() = " << classRes.size() << endl;
		for(int row = 0; row < image[c].rows; row++){
			for(int col = 0; col < image[c].cols; col++){
				if(regions.at<int>(row, col) != -1){
					int xInd = regions.at<int>(row, col) / X_RES;
					int yInd = regions.at<int>(row, col) % X_RES;
					//cout << "adding " << classRes[DRIVABLE_LABEL].at<float>(row, col) << endl;
					constraints.at<float>(yInd, xInd) += classRes[DRIVABLE_LABEL].at<float>(row, col);
					constrNorm.at<float>(yInd, xInd) += 1;
				}
			}
		}
		//cout << "normalizing constraints" << endl;
		for(int xInd = 0; xInd < X_RES; xInd++){
			for(int yInd = 0; yInd < Y_RES; yInd++){
				if(constrNorm.at<float>(yInd, xInd) != 0){
					constraints.at<float>(yInd, xInd) /= constrNorm.at<float>(yInd, xInd);
				}
			}
		}

		//cout << "building test image" << endl;
		Mat test(image[c].rows, image[c].cols, CV_32FC1);
		for(int xInd = 0; xInd < X_RES; xInd++){
			for(int yInd = 0; yInd < Y_RES; yInd++){
				selectPolygonPixels(imagePolygons[c][xInd][yInd],
									constraints.at<float>(yInd, xInd),
									test);
			}
		}
		imshow("original", image[c]);
		imshow("prob asphalt", classRes[DRIVABLE_LABEL]);
		imshow("constraints", test);
		waitKey();
	}
}

void Camera::computeImagePolygons(){
	cout << "Computing image polygons" << endl;
	//namedWindow("test");
	imagePolygons.clear();
	imagePolygons.assign(numCameras,
						vector<vector<vector<Point2f> > >(X_RES,
							vector<vector<Point2f> >(Y_RES,
									vector<Point2f>())));
	for(int c = 0; c < numCameras; c++){
		Mat rotVec;
		Rodrigues(Mat(cameraOrigGlobal[c].inv(DECOMP_SVD), Rect(0, 0, 3, 3)), rotVec);
		Mat transVec(cameraOrigGlobal[c].inv(DECOMP_SVD), Rect(3, 0, 1, 3));

		//cout << "cameraOrigGlobal[c].inv(DECOMP_SVD) = " << cameraOrigGlobal[c].inv(DECOMP_SVD) << endl;
		//cout << "rotVec = " << rotVec << endl;
		//cout << "transVec = " << transVec << endl;

		//from front left corner
		float xStart = X_RES*X_STEP;
		float xStep = -X_STEP;
		float yStart = Y_RES*Y_STEP/2;
		float yStep = -Y_STEP;
		for(int xInd = 0; xInd < X_RES; xInd++){
			for(int yInd = 0; yInd < Y_RES; yInd++){
				vector<Point3f> points;
				int ind[][2] = {{0, 0},
								{1, 0},
								{1, 1},
								{0, 1}};
				for(int i = 0; i < 4; i++){
					points.push_back(Point3f(	xStart + (xInd + ind[i][0])*xStep,
												yStart + (yInd + ind[i][1])*yStep,
												PLANE_Z));
				}
				//cout << "Polygon glob: " << endl;
				//for(int v = 0; v < points.size(); v++){
				//	cout << points[v] << endl;
				//}
				vector<Point2f> ret;
				projectPoints(	points,
								rotVec,
								transVec,
								cameraMatrix[c],
								distCoeffs[c],
								ret);
				//Mat test(480, 640, CV_8UC1, Scalar(0));
				//cout << "Polygon image: " << endl;
				//for(int v = 0; v < ret.size(); v++){
				//	cout << ret[v] << endl;
				//}
				//selectPolygonPixels(ret, 1, test);
				//imshow("test", hierClassifiers.front()->colorSegments(test));
				//waitKey();
				imagePolygons[c][xInd][yInd]= ret;
			}
		}
	}
	cout << "End computing image polygons" << endl;
}

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

cv::Mat Camera::compOrient(cv::Mat imuData){
	//cout << "Computing orientation from IMU" << endl;
	//cout << "imuData = " << imuData << endl;

	Mat ret(Mat::eye(4, 4, CV_32FC1));
	float yaw = imuData.at<float>(11)*PI/180;
	float pitch = imuData.at<float>(10)*PI/180;
	float roll = imuData.at<float>(9)*PI/180;
	//cout << "Computing Rz, Ry, Rx, yaw = " << yaw << endl;
	Matx33f Rz(	cos(yaw), sin(yaw), 0,
				-sin(yaw), cos(yaw), 0,
				0, 0, 1);
	//cout << "Rz = " << Rz << endl;
	Matx33f Ry(	cos(pitch), 0, -sin(pitch),
				0, 1, 0,
				sin(pitch), 0, cos(pitch));
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
}

bool Camera::readLine(std::ifstream& stream, Mat& data){
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
		data.push_back(val);
	}
	data = data.reshape(0, 1);
	return true;
}

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

void Camera::processDir(boost::filesystem::path dir,
						std::vector<cv::Mat>& images,
						std::vector<cv::Mat>& manualRegionsOnImages,
						std::vector<std::map<int, int> >& mapRegionIdToLabel,
						std::vector<cv::Mat>& terrains)
{
	images.clear();
	manualRegionsOnImages.clear();
	mapRegionIdToLabel.clear();
	terrains.clear();

	cout << dir.string() + "/hokuyo.data" << endl;
	ifstream hokuyoFile(dir.string() + "/hokuyo.data");
	if(!hokuyoFile.is_open()){
		throw "Error - no hokuyo file";
	}
	ifstream imuFile(dir.string() + "/imu.data");
	if(!imuFile.is_open()){
		throw "Error - no imu file";
	}
	ifstream encodersFile(dir.string() + "/encoders.data");
	if(!encodersFile.is_open()){
		throw "Error - no encoders file";
	}
	ifstream cameraFile(dir.string() + "/camera.data");
	if(!encodersFile.is_open()){
		throw "Error - no camera file";
	}
	Mat hokuyoAllPointsGlobal;
	Mat imuPrev, encodersPrev;
	Mat globalPos, curPos;
	Mat prevRot;
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

			Mat hokuyoCurPoints, hokuyoCurPointsDist, hokuyoCurPointsInt;
			char tmpChar;
			hokuyoFile >> tmpChar;
			readLine(hokuyoFile, hokuyoCurPointsDist);
			hokuyoFile >> tmpChar;
			readLine(hokuyoFile, hokuyoCurPointsInt);
			//hokuyoCurPoints = Mat(6, hokuyoCurPointsDist.cols, CV_32FC1, Scalar(1));
			hokuyoCurPoints = Mat(0, 6, CV_32FC1);

			static const float startAngle = -45*PI/180;
			static const float stepAngle = 0.25*PI/180;
			for(int i = 0; i < hokuyoCurPointsDist.cols; i++){
				if(hokuyoCurPointsDist.at<float>(i) > 500){
					Mat curPoint(1, 6, CV_32FC1);
					curPoint.at<float>(0) = cos(startAngle + stepAngle*i)*hokuyoCurPointsDist.at<float>(i);
					curPoint.at<float>(1) = 0;
					curPoint.at<float>(2) = sin(startAngle + stepAngle*i)*hokuyoCurPointsDist.at<float>(i);
					curPoint.at<float>(3) = 1;
					curPoint.at<float>(4) = hokuyoCurPointsDist.at<float>(i);
					curPoint.at<float>(5) = hokuyoCurPointsInt.at<float>(i);
					hokuyoCurPoints.push_back(curPoint);
				}
			}
			cout << endl;
			hokuyoCurPoints = hokuyoCurPoints.t();

			hokuyoCurPoints.rowRange(0, 4) = cameraOrigLaser.front().inv()*hokuyoCurPoints.rowRange(0, 4);

			//narrow hokuyo scan range 45 - 135
			//hokuyoCurPoints = hokuyoCurPoints.colRange(400, 681);

			if(imuPrev.empty()){
				readLine(imuFile, imuPrev);
				globalPos = compOrient(imuPrev)*cameraOrigImu.front();
				globalPos.copyTo(curPos);
				imuFile >> imuCurTime;
			}
			if(encodersPrev.empty()){
				readLine(encodersFile, encodersPrev);
				encodersFile >> encodersCurTime;
			}
			Mat imuCur, encodersCur;

			while(imuCurTime <= hokuyoCurTime && !imuFile.eof()){
				//cout << "Imu time: " << imuCurTime << endl;
				readLine(imuFile, imuCur);

				imuFile >> imuCurTime;
			}

			while(encodersCurTime <= hokuyoCurTime && !encodersFile.eof()){
				//cout << "Encoders time: " << encodersCurTime << endl;
				readLine(encodersFile, encodersCur);

				encodersFile >> encodersCurTime;
			}
			if(imuCur.empty()){
				imuPrev.copyTo(imuCur);
			}
			if(encodersCur.empty()){
				encodersPrev.copyTo(encodersCur);
			}

			//cout << "Computing curPos" << endl;
			//cout << "encodersCur = " << encodersCur << endl << "encodersPrev = " << encodersPrev << endl;
			Mat trans = compTrans(compOrient(imuPrev), encodersCur - encodersPrev);
			cout << "trans = " << trans << endl;
			//cout << "Computing curTrans" << endl;
			Mat curTrans = Mat(curPos, Rect(3, 0, 1, 4)) + trans;
			//cout << "Computing curRot" << endl;

			Mat curRot = compOrient(imuCur)*cameraOrigImu.front();
			curRot.copyTo(curPos);
			curTrans.copyTo(Mat(curPos, Rect(3, 0, 1, 4)));
			//cout << "trans = " << trans << endl;
			//cout << "curTrans = " << curTrans << endl;
			//cout << "curRot = " << curRot << endl;
			//cout << "curPos = " << curPos << endl;
			//cout << "globalPos.inv()*curPos = " << globalPos.inv()*curPos << endl;

			//cout << "Moving hokuyoAllPointsGlobal" << endl;
			Mat tmpAllPoints(hokuyoCurPoints.rows, hokuyoAllPointsGlobal.cols + hokuyoCurPoints.cols, CV_32FC1);
			if(!hokuyoAllPointsGlobal.empty()){
				hokuyoAllPointsGlobal.copyTo(tmpAllPoints.colRange(0, hokuyoAllPointsGlobal.cols));
			}
			//cout << "Addding hokuyoCurPoints" << endl;
			Mat hokuyoCurPointsGlobal(hokuyoCurPoints.rows, hokuyoCurPoints.cols, CV_32FC1);
			Mat tmpCurPoints = curPos*hokuyoCurPoints.rowRange(0, 4);
			tmpCurPoints.copyTo(hokuyoCurPointsGlobal.rowRange(0, 4));
			hokuyoCurPoints.rowRange(4, 6).copyTo(hokuyoCurPointsGlobal.rowRange(4, 6));
			//cout << hokuyoCurPointsGlobal.channels() << ", " << hokuyoAllPointsGlobal.channels() << endl;
			hokuyoCurPointsGlobal.copyTo(tmpAllPoints.colRange(hokuyoAllPointsGlobal.cols, hokuyoAllPointsGlobal.cols + hokuyoCurPoints.cols));
			hokuyoAllPointsGlobal = tmpAllPoints;

			//waitKey();
			Mat covarLaserCur, meanLaserCur;
			calcCovarMatrix(hokuyoCurPoints.rowRange(4, 6),
							covarLaserCur,
							meanLaserCur,
							CV_COVAR_NORMAL | CV_COVAR_SCALE | CV_COVAR_COLS,
							CV_32F);
			meanLaser = (meanLaser*numPts + meanLaserCur*hokuyoCurPoints.cols)/(numPts + hokuyoCurPoints.cols);
			covarLaser = (covarLaser*numPts + covarLaserCur*hokuyoCurPoints.cols)/(numPts + hokuyoCurPoints.cols);
			numPts += hokuyoCurPoints.cols;

			imuCur.copyTo(imuPrev);
			encodersCur.copyTo(encodersPrev);
			hokuyoFile >> hokuyoCurTime;

			curRot.copyTo(prevRot);
		}
		Mat terrain = hokuyoAllPointsGlobal.clone();
		terrain.rowRange(0, 4) = curPos.inv()*hokuyoAllPointsGlobal.rowRange(0, 4);
		terrains.push_back(terrain);

		//cout << "Displaying test image from file: " << dir.string() + string("/") + cameraImageFile.string() << endl;
		Mat image = imread(dir.string() + string("/") + cameraImageFile.filename().string());
		//rectangle(image, Point(0, 0), Point(image.cols, 100), Scalar(0, 0, 0), -1);
		images.push_back(image.clone());
		if(image.data == NULL){
			throw "Bad image file";
		}
		Mat hokuyoAllPointsCamera = curPos.inv()*hokuyoAllPointsGlobal.rowRange(0, 4);
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

		ofstream pointsFile("points/" + cameraImageFile.stem().string() + ".log");

		for(int p = 0; p < pointsImage.size(); p++){
			if(pointsImage[p].x >= 0 && pointsImage[p].x < image.cols &&
				pointsImage[p].y >= 0 && pointsImage[p].y < image.rows)
			{
				pointsFile << pointsImage[p].x << " " << pointsImage[p].y << " " << hokuyoAllPointsGlobal.at<float>(5, p) << endl;
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
		waitKey();

		/*TiXmlDocument data(	dir.string() +
							string("/") +
							cameraImageFile.stem().string() +
							string(".xml"));
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

		Mat manualRegionsOnImageCur(image.rows, image.cols, CV_32SC1, Scalar(0));
		int manualRegionsCount = 0;
		map<int, int> mapRegionIdToLabelCur;

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
			int label = -1;
			for(int i = 0; i < labels.size(); i++){
				if(labelText == labels[i]){
					label = i;
					break;
				}
			}
			if(label == -1){
				throw "No such label found";
			}

			mapRegionIdToLabelCur[++manualRegionsCount] = label;
			//cout << "Selecting polygon pixels for label " << labels[label] <<  endl;
			selectPolygonPixels(poly, manualRegionsCount, manualRegionsOnImageCur);
			//cout << "End selecting" << endl;

			pObject = pObject->NextSiblingElement("object");
		}
		mapRegionIdToLabel.push_back(mapRegionIdToLabelCur);
		manualRegionsOnImages.push_back(manualRegionsOnImageCur);*/
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

	for(int d = 0; d < dirs.size(); d++){
		std::vector<cv::Mat> tmpImages;
		std::vector<cv::Mat> tmpManualRegionsOnImages;
		std::vector<std::map<int, int> > tmpMapRegionIdToLabel;
		std::vector<cv::Mat> tmpTerrains;
		processDir(	dirs[d],
					tmpImages,
					tmpManualRegionsOnImages,
					tmpMapRegionIdToLabel,
					tmpTerrains);
		for(int i = 0; i < tmpImages.size(); i++){
			images.push_back(tmpImages[i]);
			manualRegionsOnImages.push_back(tmpManualRegionsOnImages[i]);
			mapRegionIdToLabel.push_back(tmpMapRegionIdToLabel[i]);
			terrains.push_back(tmpTerrains[i]);
		}
	}

	cout << "images.size() = " << images.size() << endl << "manualRegionsOnImages.size() = " << manualRegionsOnImages.size() << endl;

	vector<Entry> dataset;

	for(int i = 0; i < images.size(); i++){
		cout << "Segmenting" << endl;
		Mat autoRegionsOnImage = hierClassifiers.front()->segmentImage(images[i]);
		cout << "Assigning manual ids" << endl;
		//rectangle(manualRegionsOnImages[i], Point(0, 0), Point(images[i].cols, 100), Scalar(0, 0, 0), -1);
		map<int, int> assignedManualId = hierClassifiers.front()->assignManualId(autoRegionsOnImage, manualRegionsOnImages[i]);
		imshow("original", images[i]);
		imshow("segments", hierClassifiers.front()->colorSegments(autoRegionsOnImage));
		waitKey(50);
		cout << "Extracting entries" << endl;
		vector<Entry> newData = hierClassifiers.front()->extractEntries(images[i], terrains[i], autoRegionsOnImage);
		for(int e = 0; e < newData.size(); e++){
			if(mapRegionIdToLabel[i].count(assignedManualId[newData[e].imageId]) > 0){
				newData[e].label = mapRegionIdToLabel[i][assignedManualId[newData[e].imageId]];
				dataset.push_back(newData[e]);
			}
		}
	}
	ofstream dataFile("data.log");
	dataFile.precision(15);
	for(int e = 0; e < dataset.size(); e++){
		dataFile << dataset[e].label << " " << dataset[e].weight << " ";
		for(int d = 0; d < dataset[e].descriptor.cols; d++){
			dataFile << dataset[e].descriptor.at<float>(d) << " ";
		}
		dataFile << endl;
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

	if(crossValidate){
		hierClassifiers.front()->crossValidateSVMs(dataset);
	}
	hierClassifiers.front()->train(dataset, labels.size());

	if(cacheEnabled){
		saveCache("cache/cameraCache");
	}
}

void Camera::classifyFromDir(std::vector<boost::filesystem::path> dirs){
	cout << "Classifying" << endl;
	for(int l = 0; l < labels.size(); l++){
		namedWindow(labels[l]);
	}
	namedWindow("segments");
	namedWindow("original");
	namedWindow("colored");

	vector<vector<int> > classResultsPix(labels.size(), vector<int>(labels.size(), 0));
	vector<vector<int> > classResultsSeg(labels.size(), vector<int>(labels.size(), 0));

	std::vector<cv::Mat> images;
	std::vector<cv::Mat> manualRegionsOnImages;
	std::vector<std::map<int, int> > mapRegionIdToLabel;
	std::vector<cv::Mat> terrains;

	for(int d = 0; d < dirs.size(); d++){
		std::vector<cv::Mat> tmpImages;
		std::vector<cv::Mat> tmpManualRegionsOnImages;
		std::vector<std::map<int, int> > tmpMapRegionIdToLabel;
		std::vector<cv::Mat> tmpTerrains;
		processDir(	dirs[d],
					tmpImages,
					tmpManualRegionsOnImages,
					tmpMapRegionIdToLabel,
					tmpTerrains);
		for(int i = 0; i < tmpImages.size(); i++){
			images.push_back(tmpImages[i]);
			manualRegionsOnImages.push_back(tmpManualRegionsOnImages[i]);
			mapRegionIdToLabel.push_back(tmpMapRegionIdToLabel[i]);
			terrains.push_back(tmpTerrains[i]);
		}
	}

	cout << "images.size() = " << images.size() << endl << "manualRegionsOnImages.size() = " << manualRegionsOnImages.size() << endl;

	vector<Entry> dataset;

	for(int i = 0; i < images.size(); i++){
		Mat autoSegmented = hierClassifiers.front()->segmentImage(images[i]);
		map<int, int> assignedManualId = hierClassifiers.front()->assignManualId(autoSegmented, manualRegionsOnImages[i]);
		imshow("original", images[i]);
		imshow("segments", hierClassifiers.front()->colorSegments(autoSegmented));

		vector<vector<int> > curClassResultsPix(labels.size(), vector<int>(labels.size(), 0));
		vector<vector<int> > curClassResultsSeg(labels.size(), vector<int>(labels.size(), 0));

		vector<Mat> classificationResult = hierClassifiers.front()->classify(images[i], terrains[i], autoSegmented);
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
		Mat coloredOriginal(images[i].rows, images[i].cols, CV_8UC3);
		Mat bestLabels(images[i].rows, images[i].cols, CV_32SC1, Scalar(0));
		Mat bestScore(images[i].rows, images[i].cols, CV_32FC1, Scalar(-1));
		for(int l = 0; l < labels.size(); l++){
			Mat cmp;
			compare(bestScore, classificationResult[l], cmp, CMP_LE);
			bestLabels.setTo(l, cmp);
			bestScore = max(bestScore, classificationResult[l]);
		}
		for(int l = 0; l < labels.size(); l++){
			coloredOriginal.setTo(colors[l], bestLabels == l);
		}
		imshow("colored", coloredOriginal * 0.25 + images[i] * 0.75);

		set<int> counted;
		for(int r = 0; r < images[i].rows; r++){
			for(int c = 0; c < images[i].cols; c++){
				int pred = bestLabels.at<int>(r, c);
				if(mapRegionIdToLabel[i].count(assignedManualId[autoSegmented.at<int>(r, c)]) > 0){
					if(counted.count(autoSegmented.at<int>(r, c)) == 0){
						int groundTrue = mapRegionIdToLabel[i][assignedManualId[autoSegmented.at<int>(r, c)]];
						curClassResultsSeg[groundTrue][pred]++;
						counted.insert(autoSegmented.at<int>(r, c));
					}
				}
				if(mapRegionIdToLabel[i].count(manualRegionsOnImages[i].at<int>(r, c)) > 0){
					int groundTrue = mapRegionIdToLabel[i][manualRegionsOnImages[i].at<int>(r, c)];
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

		waitKey(100);
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
void Camera::cameraThread(){

}



void Camera::readSettings(TiXmlElement* settings){
	string tmp;

	if(settings->QueryIntAttribute("number", &numCameras) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of cameras";
	}
	if(settings->QueryIntAttribute("rows", &numRows) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of rows";
	}
	if(settings->QueryIntAttribute("cols", &numCols) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of cols";
	}

	cacheEnabled = true;
	TiXmlElement* pPtr = settings->FirstChildElement("cache");
	if(!pPtr){
		throw "Bad settings file - no cache setting for Camera";
	}
	pPtr->QueryBoolAttribute("enabled", &cacheEnabled);

	crossValidate = false;

	pPtr = settings->FirstChildElement("learning");
	if(!pPtr){
		throw "Bad settings file - no learning settings";
	}
	pPtr->QueryStringAttribute("dir", &tmp);
	learningDir = tmp;

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

	pPtr = settings->FirstChildElement("sensor");
	cameraOrigGlobal.resize(numCameras);
	cameraOrigLaser.resize(numCameras);
	cameraOrigImu.resize(numCameras);
	cameraMatrix.resize(numCameras);
	distCoeffs.resize(numCameras);
	hierClassifiers.resize(numCameras);
	for(int i = 0; i < numCameras; i++){
		if(!pPtr){
			throw "Bad settings file - no sensor settings";
		}
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

		cameraOrigGlobal[idx] = readMatrixSettings(pPtr, "position_global", 4, 4);
		cameraOrigLaser[idx] = readMatrixSettings(pPtr, "position_laser", 4, 4);
		cameraOrigImu[idx] = readMatrixSettings(pPtr, "position_imu", 4, 4).t();
		cameraMatrix[idx] = readMatrixSettings(pPtr, "camera_matrix", 3, 3);
		distCoeffs[idx] = readMatrixSettings(pPtr, "dist_coeffs", 1, 5);


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

	groundPlane = readMatrixSettings(settings, "ground_plane_global", 4, 1);

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

//Returns constraints map and inserts time of data from cameras fetch
const cv::Mat Camera::getConstraints(int* timestamp){

}

//CV_8UC3 2x640x480: left, right image
const std::vector<cv::Mat> Camera::getData(){
	//empty matrix
	vector<Mat> ret;
	for(int i = 0; i < cameras.size(); i++){
		cameras[i].grab();
	}
	for(int i = 0; i < cameras.size(); i++){
		Mat tmp;
		cameras[i].retrieve(tmp);
		ret.push_back(tmp);
	}
	return ret;
}

void Camera::open(std::vector<std::string> device){
	cameras.resize(device.size());
	for(int i = 0; i < device.size(); i++){
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
	cameras.clear();
}

bool Camera::isOpen(){
	return true;
}
