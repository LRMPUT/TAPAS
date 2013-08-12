/*
 * Camera.cpp
 *
 *  Created on: 08-07-2013
 *      Author: jachu
 */

//OpenCV
#include <opencv2/opencv.hpp>
//STL
#include <cmath>
#include <sstream>
//RobotsIntellect
#include "Camera.h"

/*#define CAMERA_Z 1
#define CAMERA_X_ANGLE 45
#define CAMERA_Y_ANGLE 45
#define CAMERAS_COUNT 2
#define ROWS 480
#define COLS 640*/
#define POLY_VERT 4
#define LEFT_CAMERA 0
#define RIGHT_CAMERA 1

using namespace cv;
using namespace std;

Camera::Camera(MovementConstraints* imovementConstraints, TiXmlElement* settings) :
		movementConstraints(imovementConstraints),
		cameraGrid(40)
{
	if(!settings){
		throw "Bad settings file - entry Camera not found";
	}
	readSettings(settings);

	groundPolygons.resize(numCameras);
	for(int cam = 0; cam < numCameras; cam++){
		groundPolygons[cam].resize(numRows/cameraGrid);
		for(int row = 0; row < numRows/cameraGrid; row++){
			groundPolygons[cam][row].resize(numCols/cameraGrid);
			for(int col = 0; col < numCols/cameraGrid; col++){
				groundPolygons[cam][row][col] = new Point[POLY_VERT];
			}
		}
	}
	computeGroundPolygons();
}

Camera::~Camera(){
	for(int cam = 0; cam < numCameras; cam++){
		for(int row = 0; row < numRows/cameraGrid; row++){
			for(int col = 0; col < numCols/cameraGrid; col++){
				delete[] groundPolygons[cam][row][col];
			}
		}
	}
}

void Camera::computeConstraints(std::vector<cv::Mat> image){

}

void Camera::computeGroundPolygons(){
	int rows = numRows/cameraGrid;
	int cols = numCols/cameraGrid;
	for(int im = 0; im < numCameras; im++){
		Mat cornersX(rows + 1, cols + 1, CV_32SC1);
		Mat cornersY(rows + 1, cols + 1, CV_32SC1);
		for(int nrow = 0; nrow < rows; nrow++){
			for(int ncol = 0; ncol < cols; ncol++){
				//computing top left corners
				Point3f point = computePointProjection(	Point2f((float)(ncol - cols/2) / (cols/2),
																-(float)(nrow - rows/2) / (rows/2)),
														im);
				cornersX.at<int>(nrow, ncol) = point.x;
				cornersY.at<int>(nrow, ncol) = point.y;
			}
		}
		for(int ncol = 0; ncol < cols; ncol++){
			//computing bottom left corners
			Point3f point = computePointProjection(	Point2f((float)(ncol - cols/2) / (cols/2),
															-1),
													im);
			cornersX.at<int>(rows, ncol) = point.x;
			cornersY.at<int>(rows, ncol) = point.y;
		}
		for(int nrow = 0; nrow < rows; nrow++){
			//computing top right corners
			Point3f point = computePointProjection(	Point2f(1,
															-(float)(nrow - rows/2) / (rows/2)),
													im);
			cornersX.at<int>(nrow, cols) = point.x;
			cornersY.at<int>(nrow, cols) = point.y;
		}
		//computing bottom right corner
		Point3f point = computePointProjection(	Point2f(1,
														-1),
												im);
		cornersX.at<int>(rows, cols) = point.x;
		cornersY.at<int>(rows, cols) = point.y;
		//Polygons on the ground for each image region
		for(int nrow = 0; nrow < rows; nrow++){
			for(int ncol = 0; ncol < cols; ncol++){
				groundPolygons[im][nrow][ncol][0] = Point(cornersX.at<int>(nrow, ncol), cornersY.at<int>(nrow, ncol));
				groundPolygons[im][nrow][ncol][1] = Point(cornersX.at<int>(nrow, ncol+1), cornersY.at<int>(nrow, ncol+1));
				groundPolygons[im][nrow][ncol][2] = Point(cornersX.at<int>(nrow+1, ncol+1), cornersY.at<int>(nrow+1, ncol+1));
				groundPolygons[im][nrow][ncol][3] = Point(cornersX.at<int>(nrow+1, ncol), cornersY.at<int>(nrow+1, ncol));
			}
		}
	}
}

cv::Point3f Camera::computePointProjection(cv::Point2f imPoint, int cameraInd){
	Mat point(3, 1, CV_32FC1);
	point.at<float>(0) = imPoint.x * cameraZ * tan(angleX/2);
	point.at<float>(1) = imPoint.y * cameraZ * tan(angleY/2);
	point.at<float>(2) = -cameraZ;

	Mat rot(cameraOrig[cameraInd], Rect(Point(0, 0), Point(3, 3)));
	Mat trans(cameraOrig[cameraInd], Rect(Point(3, 0), Point(4, 3)));
	point = rot * point;
	Mat planeABC(groundPlane, Rect(Point(0, 0), Point(1, 3)));
	Mat planeD(groundPlane, Rect(Point(0, 3), Point(1, 4)));
	Mat a = (-planeABC.t() * trans - planeD) / (planeABC.t() * point);
	point = trans + point * a;
	Point3f ret(point.at<float>(0), point.at<float>(1), point.at<float>(2));
	return ret;
}

void Camera::learn(cv::Mat samples, int label){

}

void Camera::learnFromDir(boost::filesystem::path dir){

}

cv::Mat Camera::classifySlidingWindow(cv::Mat image){

}

//Run as separate thread
void Camera::cameraThread(){

}

void Camera::readSettings(TiXmlElement* settings){
	if(settings->QueryIntAttribute("number", &numCameras) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of cameras";
	}
	if(settings->QueryIntAttribute("rows", &numRows) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of rows";
	}
	if(settings->QueryIntAttribute("cols", &numCols) != TIXML_SUCCESS){
		throw "Bad settings file - wrong number of cols";
	}
	if(settings->QueryIntAttribute("angle_x", &angleX) != TIXML_SUCCESS){
		throw "Bad settings file - wrong angle x";
	}
	if(settings->QueryIntAttribute("angle_y", &angleY) != TIXML_SUCCESS){
		throw "Bad settings file - wrong angle y";
	}
	if(settings->QueryIntAttribute("camera_z", &cameraZ) != TIXML_SUCCESS){
		throw "Bad settings file - wrong camera z";
	}

	cacheEnabled = true;
	TiXmlElement* pPtr = settings->FirstChildElement("cache");
	if(!pPtr){
		throw "Bad settings file - no cache setting";
	}
	pPtr->QueryBoolAttribute("enabled", &cacheEnabled);

	pPtr = settings->FirstChildElement("svm");
	if(!pPtr){
		throw "Bad settings file - no svm settings";
	}

	int svmType;
	string tmp;
	pPtr->QueryStringAttribute("type", &tmp);
	if(tmp == "C_SVC"){
		svmType = CvSVM::C_SVC;
	}
	else if(tmp == "NU_SVC"){
		svmType = CvSVM::NU_SVC;
	}
	else if(tmp == "ONE_CLASS"){
		svmType = CvSVM::ONE_CLASS;
	}
	else{
		throw "Bad settings file - wrong SVM type";
	}

	int kernelType;
	TiXmlElement* svmPtr = pPtr->FirstChildElement("kernelType");
	if(!svmPtr){
		throw "Bad settings file - no kernel type";
	}
	svmPtr->QueryStringAttribute("value", &tmp);
	if(tmp == "LINEAR"){
		kernelType = CvSVM::LINEAR;
	}
	else if(tmp == "POLY"){
		kernelType = CvSVM::POLY;
	}
	else if(tmp == "RBF"){
		kernelType = CvSVM::RBF;
	}
	else if(tmp == "SIGMOID"){
		kernelType = CvSVM::SIGMOID;
	}
	else{
		throw "Bad settings file - wrong kernel type";
	}

	int bins = 256;
	svmPtr = pPtr->FirstChildElement("bins");
	if(!svmPtr){
		throw "Bad settings file - no bins number";
	}
	svmPtr->QueryIntAttribute("value", &bins);

	double gamma = 0.5;
	svmPtr = pPtr->FirstChildElement("gamma");
	if(!svmPtr){
		throw "Bad settings file - no gamma value";
	}
	svmPtr->QueryDoubleAttribute("value", &gamma);

	double degree = 2;
	svmPtr = pPtr->FirstChildElement("degree");
	if(!svmPtr){
		throw "Bad settings file - no degree value";
	}
	svmPtr->QueryDoubleAttribute("value", &degree);

	classifyGrid = 40;
	pPtr = settings->FirstChildElement("classification");
	if(!pPtr){
		throw "Bad settings file - no classification settings";
	}
	pPtr->QueryIntAttribute("grid", &classifyGrid);

	pPtr = settings->FirstChildElement("learning");
	if(!pPtr){
		throw "Bad settings file - no learning settings";
	}
	pPtr->QueryStringAttribute("dir", &tmp);
	learningDir = tmp;

	pPtr = settings->FirstChildElement("sensor");
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
		TiXmlElement* posPtr = pPtr->FirstChildElement("position");
		if(!posPtr){
			throw "Bad settings file - no position of sensor";
		}
		stringstream tmpStr(posPtr->Value());
		cameraOrig[idx] = Mat(4, 4, CV_32FC1);
		for(int row = 0; row < 4; row++){
			for(int col = 0; col < 4; col++){
				float tmpVal;
				tmpStr >> tmpVal;
				cameraOrig[idx].at<float>(row, col) = tmpVal;
			}
		}
		pPtr = pPtr->NextSiblingElement("sensor");
	}

	groundPlane = Mat(4, 1, CV_32FC1);
	pPtr = settings->FirstChildElement("ground_plane");
	if(!pPtr){
		throw "Bad settings file - no ground plane equation";
	}
	stringstream tmpStr(pPtr->Value());
	for(int i = 0; i < 4; i++){
		float tmpVal;
		tmpStr >> tmpVal;
		groundPlane.at<float>(i) = tmpVal;
	}

	svmParams = CvSVMParams();	//default values
	svmParams.kernel_type = kernelType;
	svmParams.svm_type = svmType;
	svmParams.degree = degree;
	svmParams.gamma = gamma;
}

void Camera::readCache(boost::filesystem::path cacheFile){

}

void Camera::saveCache(boost::filesystem::path cacheFile){

}

//Returns constraints map and inserts time of data from cameras fetch
const cv::Mat Camera::getConstraints(int* timestamp){

}

//CV_8UC3 2x640x480: left, right image
const std::vector<cv::Mat> Camera::getData(){
	//empty matrix
	vector<Mat> ret;
	ret.push_back(Mat(numRows, numCols, CV_8UC3));
	ret.push_back(Mat(numRows, numCols, CV_8UC3));
	return ret;
}

void Camera::open(){

}

void Camera::close(){

}

bool Camera::isOpen(){
	return true;
}
