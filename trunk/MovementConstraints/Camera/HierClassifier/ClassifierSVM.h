/*
 * ClassifierSVM.h
 *
 *  Created on: 22-10-2013
 *      Author: jachu
 */

#ifndef CLASSIFIERSVM_H_
#define CLASSIFIERSVM_H_

//STL
#include <vector>
//OpenCV
#include <opencv2/opencv.hpp>
//TinyXML
#include <tinyxml.h>
//Boost
#include <boost/filesystem.hpp>
//LibSVM
#include <libsvm/svm.h>

#include "HierClassifier.h"
#include "Classifier.h"

class ClassifierSVM : public Classifier {
	std::vector<svm_model*> svms;
	std::vector<svm_parameter> svmsParams;
	std::vector<svm_problem> svmsProblems;
	svm_parameter svmMainParams;
	svm_node** labData;
	std::vector<double*> dataLabels;
	std::vector<int> numEntriesLabeled;

	void startup();
public:
//---------------MISCELLANEOUS----------------

	ClassifierSVM();

	/** \brief Loads settings from XML structure.

	*/
	ClassifierSVM(TiXmlElement* settings);

	virtual ~ClassifierSVM();

	/** \brief Loads settings from XML structure.

	*/
	virtual void loadSettings(TiXmlElement* settings);

	virtual void saveCache(boost::filesystem::path file);

	virtual void loadCache(boost::filesystem::path file);

//---------------COMPUTING----------------
	virtual void train(std::vector<Entry> entieties);

	virtual cv::Mat classify(cv::Mat features, cv::Mat prevOutput);
};


#endif /* CLASSIFIERSVM_H_ */
