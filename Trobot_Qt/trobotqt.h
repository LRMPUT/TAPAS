#ifndef TROBOTQT_H
#define TROBOTQT_H

#include <opencv2/opencv.hpp>

#include <QtGui/QMainWindow>
#include <QtCore/QTimer>
#include <QtGui/QKeyEvent>

#include <vector>
#include <string>

#include "ui_trobotqt.h"
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

class TrobotQt : public QMainWindow
{
	Q_OBJECT
		
	friend class CameraWindow;

public:
	TrobotQt(const char* settingsFile, QWidget *parent = 0, Qt::WFlags flags = 0);
	~TrobotQt();
protected:

private:
	Ui::TrobotQtClass ui;
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


};

#endif // TROBOTQT_H
