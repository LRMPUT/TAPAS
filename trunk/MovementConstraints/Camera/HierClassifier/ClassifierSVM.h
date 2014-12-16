/*Copyright (c) 2014, TAPAS Team (cybair [at] put.poznan.pl), Poznan University of Technology
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
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

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
//LibSVM-weights
#include "../LibSVM-weights/svm.h"

#include "HierClassifier.h"
#include "Classifier.h"

/** \brief Klasa będąca realizacją słabego klasyfikatora jako klasyfikator SVM.
 *
 */
class ClassifierSVM : public Classifier {
	/** Struktura LibSVM zawierająca model nauczonego klasyfikatora. */
	svm_model* svm;
	/** Struktura LibSVM opisująca dane uczące dla klasyfikatora. */
	svm_problem svmProblem;
	/** Struktura LibSVM z parametrami dla klasyfikatora. */
	svm_parameter svmParams;
	/** Struktura LibSVM z wektorami cech dla klasyfikatora. */
	svm_node** labData;
	/** Tablica etykiet dla danych uczących. */
	double* dataLabels;
	/** Współczynnik skalowania dla danych dla każdego wymiaru wektora cech. */
	double* scalesSub;
	/** Współczynnik skalowania dla danych dla każdego wymiaru wektora cech. */
	double* scalesDiv;

	/** Liczba przykładów uczących. */
	int numEntries;
	/** Liczba wykorzystywanych etykiet. Etykiety są numerowane kolejnymi
	 * 	liczbami całkowitymi, zaczynając od 0.
	 */
	int numLabels;
	/** Długość wektora cech. */
	int descLen;
	/** Flaga informująca czy wczytywać i zapisywać cache. */
	bool cacheEnabled;

	/** \brief Funkcja inicjująca klasyfikator.
	 *
	 */
	void startup();

	/** \brief Funkcja czyszcząca dane klasyfikatora.
	 *
	 */
	void clearData();

	/** \brief Funkcja przygotowująca opis problemu dla biblioteki LibSVM.
	 *
	 */
	void prepareProblem(const std::vector<Entry>& entries,
						const std::vector<double>& productWeights = std::vector<double>());
public:
//---------------MISCELLANEOUS----------------

	/** \brief Standardowy konstruktor.
	 *
	 */
	ClassifierSVM();

	/** \brief Konstruktor kopiujący - używać tylko dla klasyfikatorów nauczonych,
	 * 			nie dla tych z danymi wczytanymi z cache.
	 *
	 */
	ClassifierSVM(const ClassifierSVM& old);

	/** \brief Konstruktor ładujący ustawienia z pliku XML.

	*/
	ClassifierSVM(TiXmlElement* settings);

	/** \brief Funkcja tworząca kopię klasyfikatora i zwracająca do niej wskaźnik.
	 *
	 */
	virtual ClassifierSVM* copy();

	virtual ~ClassifierSVM();

	/** \brief Funkcja ładująca ustawienia z pliku XML.

	*/
	virtual void loadSettings(TiXmlElement* settings);

	/** \brief Funkcja zapisująca cache do pliku.
	 *
	 */
	virtual void saveCache(TiXmlElement* settings, boost::filesystem::path file);

	/** \brief Funkcja ładująca cache do pliku.
	 *
	 */
	virtual void loadCache(TiXmlElement* settings, boost::filesystem::path file);

//---------------COMPUTING----------------

	/** \brief Funkcja ucząca klasyfikator z możliwością przekazania zewnętrznych wag.
	 *
	 */
	virtual void train(const std::vector<Entry>& entries,
					   const std::vector<double>& productWeights = std::vector<double>());

	/** \brief Funkcja klasyfikująca.
	 * @return Macierz 1xliczba_etykiet z prawdopodobieństwem dla każdej etykiety.
	 */
	virtual cv::Mat classify(cv::Mat features);

	/** \brief Funkcja dokonująca cross validation.
	 *
	 */
	virtual void crossValidate(const std::vector<Entry>& entries);
};


#endif /* CLASSIFIERSVM_H_ */
