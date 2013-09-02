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
#include "../Robot/Robot.h"
#include "ui_trobotqt.h"
#include <fstream>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

class Recording : public QObject {
	Q_OBJECT

private:
	QTimer hokuyoTimer;
	QTimer encodersTimer;
	QTimer gpsTimer;
	QTimer imuTimer;
	//QTimer camerasTimer;
	QElapsedTimer time;

	Robot* robot;

	std::fstream file;

	Ui::TrobotQtClass* ui;

	// Saving Streams
	std::ofstream imuStream, gpsStream, encodersStream;
public:
	Recording(Ui::TrobotQtClass* iui, Robot* irobot);
	~Recording();

private slots:
	void getDataHokuyo();
	void getDataEncoders();
	void getDataGps();
	void getDataImu();
	void getDataCamera();

public slots:
	void startRec();
	void pauseResumeRec();
	void stopRec();
};

#endif /* RECORDING_H_ */
