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
	refreshTimer.start();
}

QtCamera::~QtCamera(){
	delete remoteCamera;
}

std::vector<cv::Mat> QtCamera::getFrame(){
	return debug->getCameraData();
}

void QtCamera::refresh(){
	cout << "QtCamera::refresh()" << endl;
	if(robot->isCameraOpen()){
		vector<Mat> frames = getFrame();
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
		robot->openCamera(ports);
	}
}

void QtCamera::disconnect(){
	robot->closeCamera();
}

void QtCamera::openCameraWindow(){
	remoteCamera->show();
	//QObject::connect(remoteCamera, SIGNAL(destroyed()), this, SLOT(cameraWindowClosed()));
}

void QtCamera::cameraWindowClosed(){

}

