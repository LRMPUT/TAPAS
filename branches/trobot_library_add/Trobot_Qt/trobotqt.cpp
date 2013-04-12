#include <iostream>
#include "trobotqt.h"
using namespace std;

#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480
#define PORTS_UPPER_LIMIT 100

TrobotQt::TrobotQt(QWidget *parent, Qt::WFlags flags)
	: cap(0), QMainWindow(parent, flags), remoteCamera(NULL), drive(NULL)
{
	if(!cap.isOpened()){ // check if we succeeded
        exit(-1);
	}
	ui.setupUi(this);
	std::vector<string> ports;
	trobot::SerialPort::DetectComPorts(ports, PORTS_UPPER_LIMIT);
	for(int i = 0; i < ports.size(); i++){
		portList << QString(ports[i].c_str());
	}

	remoteCamera = new CameraWindow(this);
	QObject::connect(&cameraTimer, SIGNAL(timeout()), this, SLOT(captureFrame()));
	QObject::connect(ui.cameraNewWindowButton, SIGNAL(clicked()), this, SLOT(openCameraWindow()));
	cameraTimer.setInterval(50);
	cameraTimer.start();

	QObject::connect(ui.robotDriveConnectButton, SIGNAL(clicked()), this, SLOT(openRobotDrive()));
	QObject::connect(ui.robotDriveDisconnectButton, SIGNAL(clicked()), this, SLOT(closeRobotDrive()));
	QObject::connect(ui.robotDriveSearchButton, SIGNAL(clicked()), this, SLOT(searchRobotDrive()));
	ui.robotDrivePortCombo->addItems(portList);

	imuChart = new ImuChart(&ui);
}

TrobotQt::~TrobotQt(){
	delete remoteCamera;
}

void TrobotQt::captureFrame(){
	cv::Mat tmpFrame;
	cap >> tmpFrame;
	cv::resize(tmpFrame, frame, cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT));
	if(ui.cameraLabel->isVisible() == true){
		ui.cameraLabel->setPixmap(QPixmap::fromImage(QImage(frame.ptr(), frame.cols, frame.rows, QImage::Format_RGB888)));
		ui.cameraLabel->update();
	}
	if(remoteCamera->isVisible() == true){
		remoteCamera->setFrame(frame.ptr(), frame.cols, frame.rows);
	}
}

void TrobotQt::openCameraWindow(){
	remoteCamera->show();
	//QObject::connect(remoteCamera, SIGNAL(destroyed()), this, SLOT(cameraWindowClosed()));
}

void TrobotQt::cameraWindowClosed(){

}

void TrobotQt::openRobotDrive(){
	if(ui.robotDrivePortCombo->count() != 0){
		drive = new QtRobotDrive(string(ui.robotDrivePortCombo->currentText().toAscii().data()), &ui);
	}
}

void TrobotQt::closeRobotDrive(){
	if(drive != NULL){
		delete drive;
		drive = NULL;
	}
}


void TrobotQt::searchRobotDrive(){

}
