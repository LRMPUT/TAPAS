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

/** \brief Wirtualna klasa, z której dziedziczą wszystkie słabe klasyfikatory.
 *
 */
class Classifier{

public:
	enum ClassifierType{
		SVM,
		RF
	};
protected:
	/** Typ klasyfikatora. */
	Classifier::ClassifierType classifierType;
	//TiXmlElement* curSettings;
public:
//---------------MISCELLANEOUS----------------

	/** \brief Standardowy konstruktor.
	 *
	 */
	Classifier(Classifier::ClassifierType iclassifierType);

	/** \brief Konstruktor ładujący ustawienia z pliku XML.

	*/
	Classifier(Classifier::ClassifierType iclassifierType, TiXmlElement* settings);

	virtual ~Classifier();

	/** \brief Funkcja tworząca kopię klasyfikatora i zwracająca do niej wskaźnik.
	 *
	 */
	virtual Classifier* copy() = 0;

	/** \brief Funkcja ładująca ustawienia z pliku XML.

	*/
	virtual void loadSettings(TiXmlElement* settings) = 0;

	/** \brief Funkcja zapisująca cache do pliku.
	 *
	 */
	virtual void saveCache(TiXmlElement* settings, boost::filesystem::path file) = 0;

	/** \brief Funkcja ładująca cache do pliku.
	 *
	 */
	virtual void loadCache(TiXmlElement* settings, boost::filesystem::path file) = 0;

	/** Funkcja zwracająca typ klasyfikatora.
	 *
	 */
	virtual Classifier::ClassifierType type();

//---------------COMPUTING----------------

	/** \brief Funkcja ucząca klasyfikator z możliwością przekazania zewnętrznych wag.
	 *
	 */
	virtual void train(	const std::vector<Entry>& entries,
						const std::vector<double>& productWeights = std::vector<double>()) = 0;

	/** \brief Funkcja klasyfikująca.
	 * @return Macierz 1xliczba_etykiet z prawdopodobieństwem dla każdej etykiety.
	 */
	virtual cv::Mat classify(cv::Mat features) = 0;

	/** \brief Funkcja dokonująca cross validation.
	 *
	 */
	virtual void crossValidate(const std::vector<Entry>& entries) = 0;
};


#endif /* CLASSIFIER_H_ */
