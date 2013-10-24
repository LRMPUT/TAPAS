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

using namespace std;
using namespace cv;

class ClassifierSVM : public Classifier {
	vector<svm_model> svms;
	svm_parameter svmsParams;
	void startup();
public:
//---------------MISCELLANEOUS----------------

	virtual ClassifierSVM();

	/** \brief Loads settings from XML structure.

	*/
	virtual ClassifierSVM(TiXmlElement* settings);

	virtual ~ClassifierSVM();

	/** \brief Loads settings from XML structure.

	*/
	virtual void loadSettings(TiXmlElement* settings);

	virtual void saveCache(boost::filesystem::path file);

	virtual void loadCache(boost::filesystem::path file);

//---------------COMPUTING----------------
	virtual void train(std::vector<Entity> label);

	virtual cv::Mat classify(cv::Mat features);
};


#endif /* CLASSIFIERSVM_H_ */
