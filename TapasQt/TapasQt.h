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

#ifndef TapasQt_H
#define TapasQt_H

#include <opencv2/opencv.hpp>

#include <QtGui/QMainWindow>
#include <QtCore/QTimer>
#include <QtGui/QKeyEvent>

#include <vector>
#include <string>

#include "ui_TapasQt.h"
#include "../Robot/Robot.h"
#include "../Debug/Debug.h"
#include "CameraWindow.h"
#include "QtRobotDrive.h"
#include "ImuChart.h"
#include "Recording.h"
#include "QtGps.h"
#include "QtCamera.h"
#include "QtHokuyo.h"
#include "QtEncoders.h"
#include "Calibration.h"
#include "Constraints.h"

class TapasQt : public QMainWindow
{
	Q_OBJECT
		
	friend class CameraWindow;

public:
	TapasQt(const char* settingsFile, QWidget *parent = 0, Qt::WFlags flags = 0);
	~TapasQt();
protected:

private:
	Ui::TapasQtClass ui;
	QStringList portList;
	
	Robot robot;
	Debug debug;

	QtRobotDrive* drive;

	QtEncoders* encoders;

	ImuChart* imuChart;

	QtGps* qtGps;

	QtCamera* qtCamera;

	QtHokuyo* qtHokuyo;

	Calibration* calib;

	Constraints* constr;

	Recording* recording;
private slots:
	void startRobot();

};

#endif // TapasQt_H
