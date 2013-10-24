/*
 * ClassifierSVM.cpp
 *
 *  Created on: 22-10-2013
 *      Author: jachu
 */

#include "Classifier.h"


void ClassifierSVM::startup(){
	svmsParams.svm_type = C_SVC;
	svmsParams.cache_size = 32;
	svmsParams.eps = 0.001;
	svmsParams.probability = 1;
	svmsParams.nr_weight = 0;
}

ClassifierSVM::ClassifierSVM(){
	startup();
}

/** \brief Loads settings from XML structure.

*/
ClassifierSVM::ClassifierSVM(TiXmlElement* settings){
	startup();
	loadSettings(settings);
}

ClassifierSVM::~ClassifierSVM(){

}

/** \brief Loads settings from XML structure.

*/
void ClassifierSVM::loadSettings(TiXmlElement* settings){

}

void ClassifierSVM::saveCache(boost::filesystem::path file){

}

void ClassifierSVM::loadCache(boost::filesystem::path file){

}

//---------------COMPUTING----------------
void ClassifierSVM::train(std::vector<Entity> label){

}

cv::Mat ClassifierSVM::classify(cv::Mat features){

}
