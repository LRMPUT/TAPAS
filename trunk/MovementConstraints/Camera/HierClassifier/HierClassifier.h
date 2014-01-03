/*
 * HierClassifier.h
 *
 *  Created on: 10-10-2013
 *      Author: jachu
 */

#ifndef HIERCLASSIFIER_H_
#define HIERCLASSIFIER_H_

struct Entry{
	cv::Mat descriptor;
	int label, imageId;
	double weight;
	Entry() : label(-1) {}
	Entry(int ilabel, cv::Mat idescriptor, double iweight, int iimageId = 0) :
		label(ilabel),
		weight(iweight),
		imageId(iimageId)
	{
		idescriptor.copyTo(descriptor);
	}

};

//STL
#include <vector>
#include <fstream>
//OpenCV
#include <opencv2/opencv.hpp>
//TinyXML
#include <tinyxml.h>
//Boost
#include <boost/filesystem.hpp>
//Hierarchical Classifier
#include "Classifier.h"

class HierClassifier {
	friend class Debug;

	struct WeakClassifierInfo{
		int descBeg, descEnd;	// <descBeg, descEnd)
		WeakClassifierInfo() {}
		WeakClassifierInfo(int idescBeg, int idescEnd) :
			descBeg(idescBeg),
			descEnd(idescEnd)
		{}
	};

	std::vector<WeakClassifierInfo> weakClassInfo;

	std::vector<Classifier*> weakClassifiersSet;
	int numWeakClassifiers;
	int numIterations;

	cv::Mat cameraMatrix;

	std::vector<Classifier*> classifiers;
	std::vector<WeakClassifierInfo> classifiersInfo;
	std::vector<double> weights;
	int numLabels;

	float kSegment;
	int minSizeSegment;

	bool cacheEnabled;

	cv::Mat projectPointsTo3D(	cv::Mat disparity);

	cv::Mat projectPointsTo2D(	cv::Mat _3dImage);

public:

//---------------MISCELLANEOUS----------------

	HierClassifier(cv::Mat icameraMatrix);

	/** \brief Loads settings from XML structure.

	*/
	HierClassifier(cv::Mat icameraMatrix, TiXmlElement* settings);

	~HierClassifier();

	/** \brief Loads settings from XML structure.

	*/	
	void loadSettings(TiXmlElement* settings);

	void saveCache(boost::filesystem::path file);

	void loadCache(boost::filesystem::path file);

	std::map<int, int> assignManualId(cv::Mat autoSegments, cv::Mat manualSegments);

	cv::Mat colorSegments(const cv::Mat segments);

//---------------COMPUTING----------------

	void train(const std::vector<Entry>& data,
				int inumLabels);

	/**	\brief 
		@return Matrix of probabilites of belonging to certain class.
	*/
	std::vector<cv::Mat> classify(cv::Mat image,
								  cv::Mat terrain);
	
	/** \brief Extracts entries from labeled data.

	*/
	std::vector<Entry> extractEntries(cv::Mat image,
										cv::Mat terrain,
										cv::Mat regionsOnImage);

	/** \brief Segments image basing on colorspace distance
	 * 	@return Matrix with labeled regions - each region has uniqe id
	 */
	cv::Mat segmentImage(cv::Mat image);

	void crossValidateSVMs(const std::vector<Entry>& entries);
};


#endif /* HIERCLASSIFIER_H_ */

