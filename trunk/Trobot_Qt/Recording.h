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
#include "ImuChart.h"
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
	ImuChart* imuChart;

	// Saving Streams
	std::ofstream imuStream, gpsStream;
public:
	Recording(Ui::TrobotQtClass* iui, ImuChart* iimuChart);
	~Recording();

private slots:
	void getDataHokuyo();
	void getDataEncoders();
	void getDataGps();
	void getDataImu();
	void getDataCameras();

public slots:
	void startRec(Robot* irobot);
	void pauseResumeRec();
	void stopRec();
};

#endif /* RECORDING_H_ */
