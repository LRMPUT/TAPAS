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

#ifndef CAMERA_H_
#define CAMERA_H_

//STL
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <queue>
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/viz/viz3d.hpp>
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

	bool cacheSaveEnabled;

	bool cacheLoadEnabled;

	bool learnEnabled;

	int debugLevel;

	int entryWeightThreshold;

	std::vector<boost::filesystem::path> learningDirs;

	/** \brief Flaga informująca o tym czy dokonywać cross validation.
	 *
	 */
	bool crossValidate;

	boost::filesystem::path learningDir;

	//CV_32FC1 MAP_SIZExMAP_SIZE: 0-1 chance of being occupied
	cv::Mat constraints;

	//CV_32FC1 4x4: camera origin position and orientation w.r.t. IMU coordinate system
	std::vector<cv::Mat> cameraOrigImu;

	cv::Mat imuOrigRobot;

	//CV_32FC1 4x4: camera origin position and orientation w.r.t. laser coordinate system
	std::vector<cv::Mat> cameraOrigLaser;

	/** CV_32FC1 3x3: camera matrix */
	std::vector<cv::Mat> cameraMatrix;

	/** CV_32FC1 1x5: distortion coefficients. */
	std::vector<cv::Mat> distCoeffs;

	std::vector<cv::Mat> maskIgnore;

	std::vector<cv::Mat> mapSegments;

	//boost::filesystem::path settingsFile;

	std::vector<Entry> entries;

	std::vector<std::string> labels;

	//array containing polygon vertices for all image regions
	//std::vector<std::vector<std::vector<cv::Point*> > > groundPolygons;

	void computeConstraints(std::chrono::high_resolution_clock::time_point nextCurTimestamp);

	std::vector<cv::Mat> computeMapSegments(cv::Mat curPosImuMapCenter);

	std::vector<cv::Mat> computeMapSegmentsGpu(cv::Mat curPosImuMapCenter);

	std::vector<cv::Mat> computeMapCoords(cv::Mat curPosImuMapCenter);

		std::vector<cv::Mat> computeMapCoordsGpu(cv::Mat curPosImuMapCenter);

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

	bool readLineFloat(std::ifstream& stream, cv::Mat& data);

	bool readLineInt(std::ifstream& stream, cv::Mat& data);

	void processDir(boost::filesystem::path dir,
							std::vector<cv::Mat>& images,
							std::vector<cv::Mat>& manualRegionsOnImages,
							std::vector<std::map<int, int> >& mapRegionIdToLabel,
							std::vector<cv::Mat>& terrains,
							std::vector<cv::Mat>& poses,
							std::vector<std::chrono::high_resolution_clock::time_point>& timestamps,
							std::vector<cv::Mat>& mapMoves);

	/** \brief Funkcja ucząca klasyfikator danymi z katalogu.
	 *
	 */
	void learnFromDir(std::vector<boost::filesystem::path> dirs);

	/** \brief Funkcja klasyfikująca dane z katalogu.
	 *
	 */
	void classifyFromDir(std::vector<boost::filesystem::path> dirs);

	std::chrono::high_resolution_clock::time_point curTimestamp;

	bool runThread;

	std::thread cameraThread;

	std::mutex mtxDevice;

	std::mutex mtxConstr;

	std::vector<cv::Mat> classifiedImage;

	cv::Mat sharedClassifiedImage;

	cv::Mat sharedOriginalImage;

	std::mutex mtxClassIm;

	struct ClassResult{
		std::chrono::high_resolution_clock::time_point timestamp;
		int numPixels;

		ClassResult(std::chrono::high_resolution_clock::time_point itimestamp,
						int inumPixels)
			:
				timestamp(itimestamp),
				numPixels(inumPixels)
		{}
	};

	std::queue<ClassResult> classResultsHist;

	cv::Mat classResultsData;

	//Run as separate thread
	void run();

	void readSettings(TiXmlElement* settings);

	cv::Mat readMatrixSettings(TiXmlElement* parent, const char* node, int rows, int cols);

	/** \brief Insert new data and skip dataSkipped data
	 * @dataAll data stored as cols
	 */
	void insertNewData(cv::Mat& dataAll, cv::Mat newData, int dataSkipped);

	/** \brief Assign label for each map segment relaying on pixel-wise labels
	 *
	 */
	cv::Mat assignSegmentLabels(cv::Mat pixelLabels, cv::Mat coords);

	void draw3DVis(cv::viz::Viz3d& win,
					cv::Mat coords,
					cv::Mat colors,
					cv::Mat pose,
					cv::Mat segments);

	void readCache(boost::filesystem::path cacheFile);

	void saveCache(boost::filesystem::path cacheFile);
public:
	Camera(MovementConstraints* imovementConstraints, TiXmlElement* settings);
	virtual ~Camera();

	//Inserts computed constraints into map
	void insertConstraints(cv::Mat map,
							std::chrono::high_resolution_clock::time_point curTimestampMap,
							cv::Mat mapMove);

	//CV_8UC3 2x640x480: left, right image
	const std::vector<cv::Mat> getData();

	cv::Mat getClassifiedImage();

	void open(std::vector<std::string> device);

	void close();

	bool isOpen();
};

#include "../MovementConstraints.h"

#endif /* CAMERA_H_ */
