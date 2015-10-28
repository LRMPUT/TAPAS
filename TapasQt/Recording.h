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

#ifndef RECORDING_H_
#define RECORDING_H_

#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <QtCore/QElapsedTimer>
#include "../Robot/Robot.h"
#include "../Debug/Debug.h"
#include "ui_TapasQt.h"
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
	QTimer goalDirLocalMapTimer;
	QElapsedTimer time;

	Robot* robot;
	Debug* debug;

	std::fstream file;

	Ui::TapasQtClass* ui;

	// Saving Streams
	std::ofstream imuStream, gpsStream, encodersStream, hokuyoStream, cameraStream;
	std::ofstream estimatedPosStream, goalDirLocalMapStream;
public:
	Recording(Ui::TapasQtClass* iui, Robot* irobot, Debug* idebug);
	~Recording();

private slots:
	void getDataHokuyo();
	void getDataEncoders();
	void getDataGps();
	void getDataImu();
	void getDataCamera();
	void getDataEstimatedPos();
	void getGoalDirLocalMap();

public slots:
	void startRec();
	void pauseResumeRec();
	void stopRec();
};

#endif /* RECORDING_H_ */
