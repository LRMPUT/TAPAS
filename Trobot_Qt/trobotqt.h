#ifndef TROBOTQT_H
#define TROBOTQT_H

#include <opencv2/opencv.hpp>

#include <QtGui/QMainWindow>
#include <QtCore/QTimer>
#include <QtGui/QKeyEvent>

#include <vector>
#include <string>

class TrobotQt;

#include "ui_trobotqt.h"
#include "../Robot/Robot.h"
#include "CameraWindow.h"
#include "QtRobotDrive.h"
#include "ImuChart.h"
#include "Recording.h"
#include "QtGps.h"

class TrobotQt : public QMainWindow
{
	Q_OBJECT
		
	friend class CameraWindow;

public:
	TrobotQt(QWidget *parent = 0, Qt::WFlags flags = 0);
	~TrobotQt();
protected:

private:
	Ui::TrobotQtClass ui;
	QStringList portList;
	
	Robot robot;

	cv::VideoCapture cap;
	cv::Mat frame;
	QTimer cameraTimer;
	CameraWindow* remoteCamera;
	
	QtRobotDrive* drive;

	ImuChart* imuChart;

	QtGps* qtGps;

	Recording* recording;
private slots:
	void captureFrame();
	void openCameraWindow();
	void cameraWindowClosed();
};

#endif // TROBOTQT_H
