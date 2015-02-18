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

#include <iostream>

#include "TapasQt.h"

using namespace std;

#define PORTS_UPPER_LIMIT 100

TapasQt::TapasQt(const char* settingsFile, QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags),
	  robot(settingsFile),
	  debug(&robot)
{
	cout << "TapasQt::TapasQt" << endl;

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

	QObject::connect(ui.startRobotButton, SIGNAL(clicked()), this, SLOT(startRobot()));
	cout << "TapasQt::TapasQt end" << endl;

	robot.openImu("/dev/robots/imu2");
	robot.openEncoders("/dev/robots/encoders");
	robot.openHokuyo("/dev/robots/hokuyo");
	robot.openGps("/dev/robots/gps");
	robot.openRobotsDrive("/dev/robots/driverLeft", "/dev/robots/driverRight");
	robot.openCamera(vector<string>(1, "/dev/video0"));

	cout << "End TapasQt::TapasQt" << endl;
}

TapasQt::~TapasQt(){
	cout << "~TapasQt" << endl;
	delete qtGps;
	delete drive;
	delete encoders;
	delete imuChart;
	delete qtCamera;
	delete qtHokuyo;
	delete calib;
	delete constr;
	delete recording;
	cout << "End ~TapasQt" << endl;
}

void TapasQt::startRobot(){
	robot.startCompetition();
}
