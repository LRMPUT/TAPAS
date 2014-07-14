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
#include "../Debug/Debug.h"
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
	QTimer cameraTimer;
	QTimer estimatedPosTimer;
	QElapsedTimer time;

	Robot* robot;
	Debug* debug;

	std::fstream file;

	Ui::TrobotQtClass* ui;

	// Saving Streams
	std::ofstream imuStream, gpsStream, encodersStream, hokuyoStream, cameraStream;
	std::ofstream estimatedPosStream;
public:
	Recording(Ui::TrobotQtClass* iui, Robot* irobot, Debug* idebug);
	~Recording();

private slots:
	void getDataHokuyo();
	void getDataEncoders();
	void getDataGps();
	void getDataImu();
	void getDataCamera();
	void getDataEstimatedPos();

public slots:
	void startRec();
	void pauseResumeRec();
	void stopRec();
};

#endif /* RECORDING_H_ */
