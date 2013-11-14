#include <iostream>

#include "trobotqt.h"

using namespace std;

#define PORTS_UPPER_LIMIT 100

TrobotQt::TrobotQt(const char* settingsFile, QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags),
	  robot(settingsFile),
	  debug(&robot)
{
	cout << "TrobotQt::TrobotQt" << endl;

	ui.setupUi(this);
	std::vector<string> ports;
	trobot::SerialPort::DetectComPorts(ports, PORTS_UPPER_LIMIT);
	for(int i = 0; i < ports.size(); i++){
		portList << QString(ports[i].c_str());
	}

	ui.gpsPortCombo->addItems(portList);
	qtGps = new QtGps(&ui, &robot, &debug);

	ui.robotDrivePortCombo->addItems(portList);
	drive = new QtRobotDrive(&ui, &robot, &debug);

	ui.imuPortCombo->addItems(portList);
	imuChart = new ImuChart(&ui, &robot, &debug);

	ui.cameraPortCombo->addItems(portList);
	qtCamera = new QtCamera(&ui, &robot, &debug);

	ui.hokuyoPortCombo->addItems(portList);
	qtHokuyo = new QtHokuyo(&ui, &robot, &debug);

	ui.sensorsPortCombo->addItems(portList);

	recording = new Recording(&ui, &robot, &debug);
	cout << "TrobotQt::TrobotQt end" << endl;

}

TrobotQt::~TrobotQt(){
	delete qtGps;
	delete drive;
	delete imuChart;
	delete qtCamera;
	delete recording;
}
