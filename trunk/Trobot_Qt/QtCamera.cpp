/*
 * QtCamera.cpp
 *
 *  Created on: 12-11-2013
 *      Author: jachu
 */

#include "QtCamera.h"

using namespace std;
using namespace cv;

#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480

QtCamera::QtCamera(Ui::TrobotQtClass* iui, Robot* irobot, Debug* idebug) :
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
}

QtCamera::~QtCamera(){
	delete remoteCamera;
}

std::vector<cv::Mat> QtCamera::getFrame(){
	return debug->getCameraData();
}

void QtCamera::refresh(){
	vector<Mat> frames = getFrame();


	if(ui->cameraLabel->isVisible() == true){
		Mat tmp;
		cv::resize(frames[0], tmp, cv::Size(ui->cameraLabel->width(), ui->cameraLabel->height()));
		ui->cameraLabel->setPixmap(QPixmap::fromImage(QImage(tmp.ptr(), tmp.cols, tmp.rows, QImage::Format_RGB888)));
		ui->cameraLabel->update();
	}
	if(remoteCamera->isVisible() == true){
		remoteCamera->setFrame(frames[0].ptr(), CAMERA_WIDTH, CAMERA_HEIGHT);
	}
	if(ui->calibCameraLabel->isVisible() == true){
		Mat tmp;
		cv::resize(frames[0], tmp, cv::Size(ui->calibCameraLabel->width(), ui->calibCameraLabel->height()));
		ui->calibCameraLabel->setPixmap(QPixmap::fromImage(QImage(tmp.ptr(), tmp.cols, tmp.rows, QImage::Format_RGB888)));
		ui->calibCameraLabel->update();
	}
}

void QtCamera::connect(){
	if(ui->cameraPortCombo->count() != 0){
		vector<string> ports;
		ports.push_back(ui->gpsPortCombo->currentText().toAscii().data());
		robot->openCamera(ports);
		refreshTimer.start();
	}
}

void QtCamera::disconnect(){
	refreshTimer.stop();
	robot->closeCamera();
}

void QtCamera::openCameraWindow(){
	remoteCamera->show();
	//QObject::connect(remoteCamera, SIGNAL(destroyed()), this, SLOT(cameraWindowClosed()));
}

void QtCamera::cameraWindowClosed(){

}

