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
	svm_model* svm;
	svm_problem svmProblem;
	svm_parameter svmParams;
	svm_node** labData;
	double* dataLabels;
	double* weights;
	int* labels;
	int numEntries;
	int numLabels;
	int descLen;
	bool cacheEnabled;

	void startup();
	void clearData();
public:
//---------------MISCELLANEOUS----------------

	ClassifierSVM();

	ClassifierSVM(const ClassifierSVM& old);

	/** \brief Loads settings from XML structure.

	*/
	ClassifierSVM(TiXmlElement* settings);

	virtual ClassifierSVM* copy();

	virtual ~ClassifierSVM();

	/** \brief Loads settings from XML structure.

	*/
	virtual void loadSettings(TiXmlElement* settings);

	virtual void saveCache(boost::filesystem::path file);

	virtual void loadCache(boost::filesystem::path file);

//---------------COMPUTING----------------
	virtual void train(	const std::vector<Entry>& entries,
						const std::vector<double>& dataWeights);

	virtual cv::Mat classify(cv::Mat features);

	void normalizeVect(cv::Mat& vector);
};


#endif /* CLASSIFIERSVM_H_ */
