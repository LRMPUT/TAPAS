/*
 * Classifier.cpp
 *
 *  Created on: 07-11-2013
 *      Author: jachu
 */

#include "Classifier.h"

Classifier::Classifier(Classifier::ClassifierType iclassifierType) :
	classifierType(iclassifierType)
{

}

/** \brief Loads settings from XML structure.

*/
Classifier::Classifier(Classifier::ClassifierType iclassifierType, TiXmlElement* settings) :
	classifierType(iclassifierType)
{

}

Classifier::~Classifier(){

}

Classifier::ClassifierType Classifier::type(){
	return classifierType;
}
