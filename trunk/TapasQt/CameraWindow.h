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

#ifndef CAMERA_WINDOW_H
#define CAMERA_WINDOW_H

#include <QtGui/QMainWindow>

#include "ui_CameraWindow.h"
#include "../Robot/Robot.h"
#include "../Debug/Debug.h"

class TapasQt;

class CameraWindow : public QMainWindow
{
	Q_OBJECT

	Robot* robot;
	Debug* debug;

public:
	CameraWindow(Robot* irobot, Debug* idebug, QWidget *parent = 0, Qt::WFlags flags = 0);
	~CameraWindow();
	void setFrame(uchar* data, int width, int height);
protected:
	virtual void keyPressEvent(QKeyEvent* event);
	virtual void keyReleaseEvent(QKeyEvent* event);
private:
	Ui::cameraWindow ui;
};

#endif //CAMERA_WINDOW_H
