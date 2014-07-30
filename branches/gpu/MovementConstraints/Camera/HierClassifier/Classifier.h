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

/** \brief Wirtualna klasa, z której dziedziczą wszystkie słabe klasyfikatory.
 *
 */
class Classifier{

public:
	enum ClassifierType{
		SVM
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
