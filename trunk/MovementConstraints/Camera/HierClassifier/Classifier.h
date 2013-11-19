/*
 * Classifier.h
 *
 *  Created on: 22-10-2013
 *      Author: jachu
 */

#ifndef CLASSIFIER_H_
#define CLASSIFIER_H_

class Classifier;

//STL
#include <vector>
//OpenCV
#include <opencv2/opencv.hpp>
//TinyXML
#include <tinyxml.h>
//Boost
#include <boost/filesystem.hpp>

#include "HierClassifier.h"

class Classifier{

public:
	enum ClassifierType{
		SVM
	};
protected:
	Classifier::ClassifierType classifierType;
public:
//---------------MISCELLANEOUS----------------

	Classifier(Classifier::ClassifierType iclassifierType);

	/** \brief Loads settings from XML structure.

	*/
	Classifier(Classifier::ClassifierType iclassifierType, TiXmlElement* settings);

	virtual ~Classifier();

	/** \brief Loads settings from XML structure.

	*/
	virtual void loadSettings(TiXmlElement* settings) = 0;

	virtual void saveCache(boost::filesystem::path file) = 0;

	virtual void loadCache(boost::filesystem::path file) = 0;

	virtual Classifier::ClassifierType type();

//---------------COMPUTING----------------
	virtual void train(std::vector<Entry> label) = 0;

	virtual cv::Mat classify(cv::Mat features, cv::Mat prevOutput) = 0;
};


#endif /* CLASSIFIER_H_ */
