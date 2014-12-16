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

#ifndef IMU_CHART_H
#define IMU_CHART_H

#include <QtCore/QTimer>
#include <QtCore/QString>
#include <QtGui/QColor>
#include <vector>
#include "ui_TapasQt.h"
#include "Chart.h"
#include "../Trobot/include/Imu.h"
#include "../Trobot/include/Address.h"
#include "../Robot/Robot.h"
#include "../Debug/Debug.h"

#include <vector>

#define NUM_VALUES 12

struct Value {
	Value();
	Value(int iaddr, QColor icolor, const char* iname, float ifactor, QCheckBox* icheckBox = NULL, QLineEdit* iscale = NULL){
		address = iaddr;
		color = icolor;
		name = QString(iname);
		factor = ifactor;
		checkBox = icheckBox;
		scale = iscale;
	}
	int address;
	QColor color;
	QString name;
	float factor;
	QCheckBox* checkBox;
	QLineEdit* scale;
};


class ImuChart : public QObject {
	Q_OBJECT

	Chart* chart;
	//trobot::Imu* imu;
	Robot* robot;
	Debug* debug;
	Ui::TapasQtClass* ui;
	QTimer timerRefresh, timerCollectData;
	int origX, origY;
	float scaleT;
	float scales[NUM_VALUES];

	void createChart();
	void drawAxes();
	void drawAxis(float scale, int offsetBeg, int offsetAxis, QColor color, QString unit, int orient);
	void setEnableChart(bool val);
	void setEnableStartButton(bool val);
	void compScales();
	void compOrig();
	void setChartScalesAndOrig();
	int round(float a);
public:
	ImuChart(Ui::TapasQtClass* iui, Robot* irobot, Debug* idebug);
	~ImuChart();

	// Just for dumping
	//std::vector<double> getImuData();
	//bool testConnection();
public slots:
	void start();
	void stop();
	void clear();
	void update();
	void repaint();
	void collectData();
	void connect();
	void disconnect();

};


#endif //IMU_CHART_H
