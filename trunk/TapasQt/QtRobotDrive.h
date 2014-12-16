/*
    TapasQt is a GUI for TAPAS library
    Copyright (C) 2014, TAPAS Team (cybair [at] put.poznan.pl), Poznan University of Technology

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

#ifndef QT_ROBOT_DRIVE
#define QT_ROBOT_DRIVE

//#define DRIVE_DBG

#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <string>
#include "../Robot/Robot.h"
#include "../Debug/Debug.h"
#include "ui_TapasQt.h"

enum Action {
	Nothing,
	UserDefined,
	Forward,
	Backward,
	Left,
	Right
};

class QtRobotDrive : public QObject
{

	Q_OBJECT

public:
	QtRobotDrive(Ui::TapasQtClass* iui, Robot* irobot, Debug* idebug);
	~QtRobotDrive();
	Action getState();
	bool isOpen();
private:
	QTimer refreshTimer;
	Ui::TapasQtClass* ui;
	void setButtonsEnabled(bool state);
	void calcVelSteering();

	Action driveState;
	const int speed;
	int motorVal[2];
	Robot* robot;
	Debug* debug;
public slots:
	void goForward();
	void goBackward();
	void goLeft();
	void goRight();
	void leftMotorStop();
	void rightMotorStop();
	void stop();
	void motorValChanged(int val);
	void throttleChanged(int val);
	void steeringChanged(int val);
	void openRobotDrive();
	void closeRobotDrive();
	void updateState();
};

#endif //QT_ROBOT_DRIVE
