#include <iostream>

#include "trobotqt.h"

using namespace std;

#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480
#define PORTS_UPPER_LIMIT 100

TrobotQt::TrobotQt(const char* settingsFile, QWidget *parent, Qt::WFlags flags)
	: cap(0), QMainWindow(parent, flags), remoteCamera(NULL),
	  robot(settingsFile), debug(&robot)
{
	cout << "TrobotQt::TrobotQt" << endl;
	if(!cap.isOpened()){ // check if we succeeded
        cout << "No camera detected" << endl;
		//exit(-1);
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

	ui.gpsPortCombo->addItems(portList);
	qtGps = new QtGps(&ui, &robot, &debug);

	ui.robotDrivePortCombo->addItems(portList);
	drive = new QtRobotDrive(&ui, &robot, &debug);

	ui.imuPortCombo->addItems(portList);
	imuChart = new ImuChart(&ui, &robot, &debug);

	recording = new Recording(&ui, &robot, &debug);
	cout << "TrobotQt::TrobotQt end" << endl;

}

TrobotQt::~TrobotQt(){
	delete remoteCamera;
	delete drive;
	delete imuChart;
}

void TrobotQt::captureFrame(){
	if(cap.isOpened()){
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
}

void TrobotQt::openCameraWindow(){
	remoteCamera->show();
	//QObject::connect(remoteCamera, SIGNAL(destroyed()), this, SLOT(cameraWindowClosed()));
}

void TrobotQt::cameraWindowClosed(){

}
