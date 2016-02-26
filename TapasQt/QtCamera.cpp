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

#include "QtCamera.h"

using namespace std;
using namespace cv;

#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480

QtCamera::QtCamera(Ui::TapasQtClass* iui, Robot* irobot, Debug* idebug) :
	ui(iui),
	robot(irobot),
	debug(idebug)
{
	QObject::connect(ui->cameraConnectButton, SIGNAL(clicked()), this, SLOT(connect()));
	QObject::connect(ui->cameraDisconnectButton, SIGNAL(clicked()), this, SLOT(disconnect()));
	QObject::connect(&refreshTimer, SIGNAL(timeout()), this, SLOT(refresh()));

	remoteCamera = new CameraWindow(robot, debug);
	QObject::connect(ui->cameraNewWindowButton, SIGNAL(clicked()), this, SLOT(openCameraWindow()));

	refreshTimer.setInterval(100);
	refreshTimer.start();
}

QtCamera::~QtCamera(){
	delete remoteCamera;
}

std::vector<cv::Mat> QtCamera::getFrame(){
	return debug->getCameraData();
}

void QtCamera::refresh(){
//	cout << "QtCamera::refresh()" << endl;
	vector<Mat> frames = getFrame();
	if(!frames[0].empty()){
		Mat frameDisp;

		cvtColor(frames[0], frameDisp, CV_BGR2RGB);

		if(ui->cameraLabel->isVisible() == true){
			Mat tmp;
			cv::resize(frameDisp, tmp, cv::Size(ui->cameraLabel->width(), ui->cameraLabel->height()));
			ui->cameraLabel->setPixmap(QPixmap::fromImage(QImage(tmp.ptr(), tmp.cols, tmp.rows, QImage::Format_RGB888)));
			ui->cameraLabel->update();
		}
		if(remoteCamera->isVisible() == true){
			remoteCamera->setFrame(frameDisp.ptr(), CAMERA_WIDTH, CAMERA_HEIGHT);
		}
		if(ui->calibCameraLabel->isVisible() == true){
			Mat tmp;
			cv::resize(frameDisp, tmp, cv::Size(ui->calibCameraLabel->width(), ui->calibCameraLabel->height()));
			ui->calibCameraLabel->setPixmap(QPixmap::fromImage(QImage(tmp.ptr(), tmp.cols, tmp.rows, QImage::Format_RGB888)));
			ui->calibCameraLabel->update();
		}
	}
}

void QtCamera::connect(){
	if(ui->cameraPortCombo->count() != 0){
		vector<string> ports;
		ports.push_back(ui->gpsPortCombo->currentText().toAscii().data());
	}
}

void QtCamera::disconnect(){
}

void QtCamera::openCameraWindow(){
	remoteCamera->show();
	//QObject::connect(remoteCamera, SIGNAL(destroyed()), this, SLOT(cameraWindowClosed()));
}

void QtCamera::cameraWindowClosed(){

}

