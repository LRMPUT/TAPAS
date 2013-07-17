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

using namespace cv;
using namespace std;

Camera::Camera(MovementConstraints* imovementConstraints) : movementConstraints(imovementConstraints) {

}

Camera::~Camera(){

}

void Camera::computeConstraints(std::vector<cv::Mat> image){
	for(int im = 0; im < CAMERAS_COUNT; im++){
		int rows = image[im].rows;
		int cols = image[im].cols;
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
		//TODO Rysowanie trapezów na mapie
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

//CV_8UC3 2x640x480: left, right image
cv::Mat Camera::getData(){
	//empty matrix
	int size[] = {2, 640, 480};
	return Mat(3, size, CV_8UC3);
}

void Camera::open(){

}

void Camera::close(){

}

bool Camera::isOpen(){
	return true;
}
