/*
 * Recording.h
 *
 *  Created on: 01-07-2013
 *      Author: robots
 */

#ifndef RECORDING_H_
#define RECORDING_H_

#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <QtCore/QElapsedTimer>
#include "../Trobot/include/Imu.h"
#include "../Trobot/include/RobotDrive.h"
#include "../PositionEstimation/GPS/GPS.h"
#include <urg_c/urg_sensor.h>
#include "ui_trobotqt.h"
#include <fstream>

class Recording : public QObject {
	Q_OBJECT

private:
	QTimer hokuyoTimer;
	QTimer encodersTimer;
	QTimer gpsTimer;
	QTimer imuTimer;
	//QTimer camerasTimer;
	QElapsedTimer time;

	trobot::Imu* imu;
	trobot::RobotDrive* drive;
	GPS* gps;
	urg_t* hokuyo;

	std::fstream file;

	Ui::TrobotQtClass* ui;
public:
	Recording(Ui::TrobotQtClass* iui);
	~Recording();

private slots:
	void getDataHokuyo();
	void getDataEncoders();
	void getDataGps();
	void getDataImu();
	void getDataCameras();

public slots:
	void startRec(trobot::Imu* imu, trobot::RobotDrive* drive, GPS* gps, urg_t* hokuyo);
	void pauseResumeRec();
	void stopRec();
};

#endif /* RECORDING_H_ */
