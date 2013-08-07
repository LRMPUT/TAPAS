/*
 * Camera.cpp
 *
 *  Created on: 08-07-2013
 *      Author: jachu
 */

#include <opencv2/opencv.hpp>
#include <cmath>
#include "Camera.h"

#define CAMERA_Z 1
#define CAMERA_X_ANGLE 45
#define CAMERA_Y_ANGLE 45
#define CAMERAS_COUNT 2
#define ROWS 480
#define COLS 640
#define POLY_VERT 4

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

	groundPolygons.resize(CAMERAS_COUNT);
	for(int cam = 0; cam < CAMERAS_COUNT; cam++){
		groundPolygons[cam].resize(ROWS/cameraGrid);
		for(int row = 0; row < ROWS/cameraGrid; row++){
			groundPolygons[cam][row].resize(COLS/cameraGrid);
			for(int col = 0; col < COLS/cameraGrid; col++){
				groundPolygons[cam][row][col] = new Point[POLY_VERT];
			}
		}
	}
	computeGroundPolygons();
}

Camera::~Camera(){
	for(int cam = 0; cam < CAMERAS_COUNT; cam++){
		for(int row = 0; row < ROWS/cameraGrid; row++){
			for(int col = 0; col < COLS/cameraGrid; col++){
				delete[] groundPolygons[cam][row][col];
			}
		}
	}
}

void Camera::computeConstraints(std::vector<cv::Mat> image){

}

void Camera::computeGroundPolygons(){
	int rows = ROWS/cameraGrid;
	int cols = COLS/cameraGrid;
	for(int im = 0; im < CAMERAS_COUNT; im++){
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
	point.at<float>(0) = imPoint.x * CAMERA_Z * tan(CAMERA_X_ANGLE/2);
	point.at<float>(1) = imPoint.y * CAMERA_Z * tan(CAMERA_Y_ANGLE/2);
	point.at<float>(2) = -CAMERA_Z;

	Mat rot(cameraOrig[cameraInd], Rect(0, 0, 2, 2));
	Mat trans(cameraOrig[cameraInd], Rect(0, 3, 2, 3));
	point = rot * point;
	Mat planeABC(groundPlane, Rect(0, 0, 2, 0));
	Mat planeD(groundPlane, Rect(3, 0, 3, 0));
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
	settings->QueryIntAttribute("number", &numCameras);
	settings->QueryIntAttribute("rows", &numRows);
	settings->QueryIntAttribute("cols", &numCols);
	settings->QueryIntAttribute("angle_x", &angleX);
	settings->QueryIntAttribute("angle_y", &angleY);

	TiXmlElement pPtr = settings->FirstChildElement("cache");
	pPtr.QueryBoolAttribute("enabled", &cacheEnabled);

	pPtr = settings->FirstChildElement("svm");
	pPtr
	//TODO dokończyć wczytywanie ustawień
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
	ret.push_back(Mat(ROWS, COLS, CV_8UC3));
	ret.push_back(Mat(ROWS, COLS, CV_8UC3));
	return ret;
}

void Camera::open(){

}

void Camera::close(){

}

bool Camera::isOpen(){
	return true;
}
