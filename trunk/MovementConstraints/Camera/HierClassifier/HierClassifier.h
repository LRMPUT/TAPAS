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
	int label;
	Entry() : label(-1) {}
	Entry(int ilabel, cv::Mat idescriptor) : label(ilabel){
		idescriptor.copyTo(descriptor);
	}

};

struct Child{
	std::vector<int> childs;
	cv::Mat decisionMatrix;
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

/*class Region{
	int label;
	std::vector<cv::Point2f> polygon;
	Region() : label(-1) {}
	Region(int ilabel, std::vector<cv::Point2f> ipolygon) : label(ilabel), polygon(ipolygon) {}
};*/

class HierClassifier {
	friend class Debug;

	cv::Mat cameraMatrix;

	std::vector<Classifier*> classifiers;
	std::vector<std::vector<int> > childs;

	cv::Mat projectPointsTo3D(cv::Mat disparity);
	cv::Mat projectPointsTo2D(cv::Mat _3dImage);
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

//---------------COMPUTING----------------

	void train(std::vector<Entry> data);

	/**	\brief 
		@return Matrix of probabilites of belonging to certain class.
	*/
	std::vector<cv::Mat> classify(cv::Mat image,
								  cv::Mat terrain);
	
	/** \brief Extracts entities from labeled data.

	*/
	std::vector<Entry> extractEntities(cv::Mat image,
										cv::Mat terrain,
										cv::Mat regionsOnImage);

	/** \brief Segments image basing on colorspace distance
	 * 	@return Matrix with labeled regions - each region has uniqe id
	 */
	cv::Mat segmentImage(cv::Mat image);


};


#endif /* HIERCLASSIFIER_H_ */

