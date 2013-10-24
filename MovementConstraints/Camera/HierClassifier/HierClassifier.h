/*
 * HierClassifier.h
 *
 *  Created on: 10-10-2013
 *      Author: jachu
 */

#ifndef HIERCLASSIFIER_H_
#define HIERCLASSIFIER_H_

//STL
#include <vector>
#include <fstream>
//OpenCV
#include <opencv2/opencv.hpp>
//TinyXML
#include <tinyxml.h>
//Boost
#include <boost/filesystem.hpp>

struct Entity{
	cv::Mat descriptor;
	int label;
	std::vector<cv::Point2f> region;
	Entity() : label(-1) {}
	Entity(int ilabel, cv::Mat idescriptor) : label(ilabel){
		idescriptor.copyTo(descriptor);
	}

}

class Region{
	int label;
	std::vector<cv::Point2f> polygon;
	Region() : label(-1) {}
	Region(int ilabel, std::vector<cv::Point2f> ipolygon) : label(ilabel), polygon(ipolygon) {}
};

class HierClassifier {

public:

//---------------MISCELLANEOUS----------------

	HierClassifier();

	/** \brief Loads settings from XML structure.

	*/
	HierClassifier(TiXmlElement* settings);	

	~HierClassifier();

	/** \brief Loads settings from XML structure.

	*/	
	void loadSettings(TiXmlElement* settings);

	void saveCache(boost::filesystem::path file);

	void loadCache(boost::filesystem::path file);

//---------------COMPUTING----------------

	void train(std::vector<Entity> data);

	/**	\brief 
		@return Matrix of probabilites of belonging to certain class.
	*/
	std::vector<cv::Mat> classify(cv::Mat image, cv::Mat terrain);
	
	/** \brief Extracts entities from labeled data.

	*/
	std::vector<Entity> extractEntities(cv::Mat image,
										cv::Mat terrain,
										std::vector<Region> regionsOnImage);

	std::vector<Region> segmentImage(cv::Mat image);


};


#endif /* HIERCLASSIFIER_H_ */

