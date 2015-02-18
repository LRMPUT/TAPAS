/*
    TapasQt is a GUI for TAPAS library
    Copyright (C) 2014, TAPAS Team:
	-Michal Nowicki (michal.nowicki@put.poznan.pl),
	-Jan Wietrzykowski (jan.wietrzykowski@cie.put.poznan.pl).
	Poznan University of Technology

    This library is free software; you can redistribute it and/or modify it under
    the terms of the GNU Lesser General Public License as published by the
    Free Software Foundation; either version 2.1 of the License, or (at your option)
    any later version.

    This library is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
    details.

    You should have received a copy of the GNU Lesser General Public License along
    with this library; if not, write to the Free Software Foundation, Inc.,
    59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
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

Calibration::Calibration(Ui::TapasQtClass* iui, Debug* idebug) :
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

	ofstream imuOut((imuFile + QString("%1").arg(index, 3, 10, QChar('0')) + imuExt).toAscii().data());
	imuOut.precision(15);
	for(int i = 0; i < 10; i++){
		Mat data = debug->getImuData();
		for(int j = 0; j < 3; j++){
			imuOut << data.at<float>(j, 0) << " ";
		}
		imuOut << endl;
		usleep(100000);
	}
	imuOut.close();
	index++;
	ui->calibIndexLabel->setText(QString("%1").arg(index, 3, 10, QChar('0')));
}

void Calibration::reset(){
	index = 1;
	ui->calibIndexLabel->setText(QString("%1").arg(index, 3, 10, QChar('0')));
}
