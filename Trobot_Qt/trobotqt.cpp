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

	ui.robotDriversLeftPortCombo->addItems(portList);
	ui.robotDriversRightPortCombo->addItems(portList);
	drive = new QtRobotDrive(&ui, &robot, &debug);

	ui.encodersPortCombo->addItems(portList);
	encoders = new QtEncoders(&ui, &robot, &debug);

	ui.imuPortCombo->addItems(portList);
	imuChart = new ImuChart(&ui, &robot, &debug);

	ui.cameraPortCombo->addItems(portList);
	qtCamera = new QtCamera(&ui, &robot, &debug);

	ui.hokuyoPortCombo->addItems(portList);
	qtHokuyo = new QtHokuyo(&ui, &robot, &debug);

	calib = new Calibration(&ui, &debug);

	ui.sensorsPortCombo->addItems(portList);

	constr = new Constraints(&ui, &debug);

	recording = new Recording(&ui, &robot, &debug);

	QObject::connect(ui.homologationButton, SIGNAL(clicked()), this, SLOT(startHomologation()));
	cout << "TrobotQt::TrobotQt end" << endl;

	robot.openImu("/dev/robots/imu2");
	robot.openEncoders("/dev/robots/encoders");
	robot.openHokuyo("/dev/robots/hokuyo");
	robot.openGps("/dev/robots/gps");
	robot.openRobotsDrive("/dev/robots/driverLeft", "/dev/robots/driverRight");
	//robot.openCamera(vector<string>(1, "/dev/video0"));

	cout << "End TrobotQt::TrobotQt" << endl;
}

TrobotQt::~TrobotQt(){
	cout << "~TrobotQt" << endl;
	delete qtGps;
	delete drive;
	delete imuChart;
	delete qtCamera;
	delete recording;
	cout << "End ~TrobotQt" << endl;
}

void TrobotQt::startHomologation(){
	robot.homologation();
}
