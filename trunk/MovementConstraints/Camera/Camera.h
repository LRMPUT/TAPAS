/*
 * Camera.h
 *
 *  Created on: 01-07-2013
 *      Author: jachu
 */

#ifndef CAMERA_H_
#define CAMERA_H_

//STL
#include <string>
#include <vector>
//OpenCV
#include <opencv2/opencv.hpp>
//Boost
#include <boost/filesystem.hpp>
//TinyXML
#include <tinyxml.h>
//Robots Intellect
#include "HierClassifier/HierClassifier.h"

class MovementConstraints;
class Debug;

class Camera {
	friend class Debug;

	//Parent MovementConstraints class
	MovementConstraints* movementConstraints;

	std::vector<HierClassifier*> hierClassifiers;

	int numCameras, numRows, numCols;

	std::vector<cv::VideoCapture> cameras;

	bool cacheEnabled;

	boost::filesystem::path learningDir;

	//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied, robot's position (MAP_SIZE/2, 0)
	cv::Mat constraints;

	//CV_32FC1 4x4: camera origin position and orientation w.r.t. global coordinate system
	std::vector<cv::Mat> cameraOrigGlobal;

	//CV_32FC1 4x4: camera origin position and orientation w.r.t. laser coordinate system
	std::vector<cv::Mat> cameraOrigLaser;

	//CV_32FC1 3x3: camera matrix
	std::vector<cv::Mat> cameraMatrix;

	//CV_32FC1 1x5: distortion coefficients
	std::vector<cv::Mat> distCoeffs;

	//CV_32FC1 4x1: ground plane equation [A, B, C, D]'
	cv::Mat groundPlane;

	int currentTimeStamp;

	boost::filesystem::path settingsFile;

	/*CvSVM svm;

	CvSVMParams svmParams;

	int bins;*/

	std::vector<Entry> entries;

	std::vector<std::string> labels;

	//array containing polygon vertices for all image regions
	//std::vector<std::vector<std::vector<cv::Point*> > > groundPolygons;

	void computeConstraints(std::vector<cv::Mat> image);

	/*void computeGroundPolygons();

	cv::Point3f computePointProjection(cv::Point2f imPoint, int cameraInd);

	void addToLearnDatabase(cv::Mat samples, int label);

	void clearLearnDatabase();

	void learn();

	cv::Mat selectPolygonPixels(std::vector<cv::Point2i> polygon, const cv::Mat& image);*/

	void learnFromDir(boost::filesystem::path dir);

	/*cv::Mat classifySlidingWindow(cv::Mat image);

	void GenerateColorHistHSVGpu(
			const cv::gpu::GpuMat& ImageH,
			const cv::gpu::GpuMat& ImageS,
			cv::gpu::GpuMat& result,
			cv::gpu::GpuMat& buf);*/

	//Run as separate thread
	void cameraThread();

	void readSettings(TiXmlElement* settings);

	cv::Mat readMatrixSettings(TiXmlElement* parent, const char* node, int rows, int cols);

	void readCache(boost::filesystem::path cacheFile);

	void saveCache(boost::filesystem::path cacheFile);
public:
	Camera(MovementConstraints* imovementConstraints, TiXmlElement* settings);
	virtual ~Camera();

	//Returns constraints map and inserts time of data from cameras fetch
	const cv::Mat getConstraints(int* timestamp);

	//CV_8UC3 2x640x480: left, right image
	const std::vector<cv::Mat> getData();

	void open(std::vector<std::string> device);

	void close();

	bool isOpen();
};

#include "../MovementConstraints.h"

#endif /* CAMERA_H_ */
