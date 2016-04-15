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

#ifndef QTHOKUYO_H_
#define QTHOKUYO_H_

//Qt
#include <QtCore/QObject>
#include <QtCore/QTimer>
//OpenCV
#include <opencv2/opencv.hpp>
//RobotsIntellect
#include "../Robot/Robot.h"
#include "../Debug/Debug.h"
//Trobot Qt
#include "ui_TapasQt.h"
//ROS
#include "ros/ros.h"
#include "TAPAS/OpenPort.h"
#include "std_srvs/Empty.h"


class QtHokuyo : public QObject
{
	Q_OBJECT

	Ui::TapasQtClass* ui;
	Robot* robot;
	Debug* debug;

	QPixmap map;
	QTimer refreshTimer;

	ros::NodeHandle nh;
	ros::ServiceClient openHokuyoClient;
	ros::ServiceClient closeHokuyoClient;

public:
	QtHokuyo(Ui::TapasQtClass* iui, Robot* irobot, Debug* idebug);
	virtual ~QtHokuyo();

	cv::Mat getData();

public slots:
	void refresh();
	void connect();
	void disconnect();
};


#endif /* QTHOKUYO_H_ */
