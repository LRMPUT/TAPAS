/*
 * Camera.h
 *
 *  Created on: 01-07-2013
 *      Author: jachu
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <vector>
#include <opencv2/opencv.hpp>

class MovementConstraints;

class Camera {
	//Parent MovementConstraints class
	MovementConstraints* movementConstraints;

	//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robot's position (MAP_SIZE/2, 0)
	cv::Mat constraints;

	//CV_32FC1 4x4: camera origin position and orientation
	cv::Mat cameraOrig[2];

	//CV_32FC1 4x1: ground plane equation [A, B, C, D]'
	cv::Mat groundPlane;

	//Grid of image classification
	int cameraGrid;

	//array containing polygon vertices for all image regions
	std::vector<std::vector<std::vector<cv::Point*> > > groundPolygons;

	void computeConstraints(std::vector<cv::Mat> image);

	void computeGroundPolygons();

	cv::Point3f computePointProjection(cv::Point2f imPoint, int cameraInd);
public:
	Camera(MovementConstraints* imovementConstraints);
	virtual ~Camera();

	//Returns constraints map and inserts time of data from cameras fetch
	const cv::Mat getConstraints(int* timestamp);

	//CV_8UC3 2x640x480: left, right image
	const std::vector<cv::Mat> getData();

	void open();

	void close();

	bool isOpen();
};

#include "../MovementConstraints.h"

#endif /* CAMERA_H_ */
