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
