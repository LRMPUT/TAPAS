/*
 * Calibration.cpp
 *
 *  Created on: 15-11-2013
 *      Author: jachu
 */

//STL
#include <iostream>
#include <vector>
#include <fstream>
//Qt
#include <QtCore/QString>
//OpenCV
#include <opencv2/opencv.hpp>
//Trobot Qt
#include "Calibration.h"

using namespace std;
using namespace cv;

Calibration::Calibration(Ui::TrobotQtClass* iui, Debug* idebug) :
	ui(iui),
	debug(idebug),
	index(1)
{
	QObject::connect(ui->calibGetDataButton, SIGNAL(clicked()), this, SLOT(getData()));
	QObject::connect(ui->calibResetButton, SIGNAL(clicked()), this, SLOT(reset()));
}

Calibration::~Calibration(){

}

void Calibration::getData(){
	Mat hokuyoData = debug->getHokuyoData();
	vector<Mat> cameraData = debug->getCameraData();
	const QString hokuyoFile("data/hokuyo");
	const QString hokuyoExt(".log");
	const QString cameraFile("data/camera");
	const QString cameraExt(".jpg");
	const QString imuFile("data/imu");
	const QString imuExt(".log");

	ofstream hokuyoOut((hokuyoFile + QString("%1").arg(index, 3, 10, QChar('0')) + hokuyoExt).toAscii().data());
	for(int i = 0; i < hokuyoData.cols; i++){
		hokuyoOut << hokuyoData.at<int>(2, i) << endl; // " " << hokuyoData.at<int>(3, i) << endl;
	}
	hokuyoOut.close();

	imwrite((cameraFile + QString("%1").arg(index, 3, 10, QChar('0')) + cameraExt).toAscii().data(), cameraData[0]);

	/*ofstream imuOut((imuFile + QString("%1").arg(index, 3, 10, QChar('0')) + imuExt).toAscii().data());
	for(int i = 0; i < 10; i++){
		Mat data = debug->getImuData();
		for(int j = 0; j < 3; j++){
			imuOut << data.at<float>(j, 0) << " ";
		}
		imuOut << endl;
		usleep(100000);
	}
	imuOut.close();*/
	index++;
	ui->calibIndexLabel->setText(QString("%1").arg(index, 3, 10, QChar('0')));
}

void Calibration::reset(){
	index = 1;
	ui->calibIndexLabel->setText(QString("%1").arg(index, 3, 10, QChar('0')));
}
