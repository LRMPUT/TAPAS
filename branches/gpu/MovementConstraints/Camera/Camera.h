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
#include <chrono>
#include <thread>
#include <mutex>
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

/** \brief Klasa zarządzająca klasyfikatorami i kamerami, przystosowana do obsługi
 * 			wielu kamer jednocześnie.
 */
class Camera {
	friend class Debug;

	//Parent MovementConstraints class
	MovementConstraints* movementConstraints;

	std::vector<HierClassifier*> hierClassifiers;

	int numCameras, numRows, numCols;

	std::vector<cv::VideoCapture> cameras;

	bool cacheEnabled;

	/** \brief Flaga informująca o tym czy dokonywać cross validation.
	 *
	 */
	bool crossValidate;

	boost::filesystem::path learningDir;

	//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied
	cv::Mat constraints;

	//CV_32FC1 4x4: camera origin position and orientation w.r.t. IMU coordinate system
	std::vector<cv::Mat> cameraOrigImu;

	cv::Mat imuOrigGlobal;

	//CV_32FC1 4x4: camera origin position and orientation w.r.t. laser coordinate system
	std::vector<cv::Mat> cameraOrigLaser;

	/** CV_32FC1 3x3: camera matrix */
	std::vector<cv::Mat> cameraMatrix;

	/** CV_32FC1 1x5: distortion coefficients. */
	std::vector<cv::Mat> distCoeffs;

	//CV_32FC1 4x1: ground plane equation [A, B, C, D]'
	cv::Mat groundPlane;

	std::vector<cv::Mat> mapSegments;

	//boost::filesystem::path settingsFile;

	std::vector<Entry> entries;

	std::vector<std::string> labels;

	//array containing polygon vertices for all image regions
	//std::vector<std::vector<std::vector<cv::Point*> > > groundPolygons;

	void computeConstraints(std::chrono::high_resolution_clock::time_point nextCurTimestamp);

	void computeMapSegments(cv::Mat curPosImuMapCenter);

	std::vector<cv::Point2f> computePointProjection(const std::vector<cv::Point3f>& imPoint,
													int cameraInd);

	std::vector<cv::Point3f> computePointReprojection(	const std::vector<cv::Point2f>& imPoint,
														int cameraInd);

	/*void addToLearnDatabase(cv::Mat samples, int label);

	void clearLearnDatabase();

	void learn();*/

	/** \brief Funkcja rysująca na obrazie wielobok i wypełniająca go wartością regionId.
	 *
	 */
	int selectPolygonPixels(std::vector<cv::Point2i> polygon,
							float regionId,
							cv::Mat& regionsOnImage);

	/** \brief Funkcja rysująca na obrazie wielobok i wypełniająca go wartością regionId.
	 *
	 */
	int selectPolygonPixels(std::vector<cv::Point2f> polygon,
							float regionId,
							cv::Mat& regionsOnImage);

	cv::Mat compOrient(cv::Mat imuData);

	cv::Mat compTrans(	cv::Mat orient,
						cv::Mat encodersDiff);

	bool readLine(std::ifstream& stream, cv::Mat& data);

	void processDir(boost::filesystem::path dir,
							std::vector<cv::Mat>& images,
							std::vector<cv::Mat>& manualRegionsOnImages,
							std::vector<std::map<int, int> >& mapRegionIdToLabel,
							std::vector<cv::Mat>& terrains);

	/** \brief Funkcja ucząca klasyfikator danymi z katalogu.
	 *
	 */
	void learnFromDir(std::vector<boost::filesystem::path> dirs);

	/** \brief Funkcja klasyfikująca dane z katalogu.
	 *
	 */
	void classifyFromDir(boost::filesystem::path dir);

	/*cv::Mat classifySlidingWindow(cv::Mat image);

	void GenerateColorHistHSVGpu(
			const cv::gpu::GpuMat& ImageH,
			const cv::gpu::GpuMat& ImageS,
			cv::gpu::GpuMat& result,
			cv::gpu::GpuMat& buf);*/

	std::chrono::high_resolution_clock::time_point curTimestamp;

	bool runThread;

	std::thread cameraThread;

	std::mutex mtxConstr;

	std::vector<cv::Mat> classifiedImage;

	cv::Mat sharedClassifiedImage;

	cv::Mat sharedOriginalImage;

	std::mutex mtxClassIm;

	//Run as separate thread
	void run();

	void readSettings(TiXmlElement* settings);

	cv::Mat readMatrixSettings(TiXmlElement* parent, const char* node, int rows, int cols);

	void readCache(boost::filesystem::path cacheFile);

	void saveCache(boost::filesystem::path cacheFile);
public:
	Camera(MovementConstraints* imovementConstraints, TiXmlElement* settings);
	virtual ~Camera();

	//Inserts computed constraints into map
	void insertConstraints(cv::Mat map);

	//CV_8UC3 2x640x480: left, right image
	const std::vector<cv::Mat> getData();

	cv::Mat getClassifiedImage();

	void open(std::vector<std::string> device);

	void close();

	bool isOpen();
};

#include "../MovementConstraints.h"

#endif /* CAMERA_H_ */
