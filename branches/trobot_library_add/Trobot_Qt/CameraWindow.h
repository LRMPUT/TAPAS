#ifndef CAMERA_WINDOW_H
#define CAMERA_WINDOW_H

class CameraWindow;

#include "trobotqt.h"
#include <QtGui/QMainWindow>

#include "../Trobot/include/RobotDrive.h"
#include "ui_CameraWindow.h"

class CameraWindow : public QMainWindow
{
	Q_OBJECT

public:
	CameraWindow(TrobotQt* imainWindow, QWidget *parent = 0, Qt::WFlags flags = 0);
	~CameraWindow();
	void setFrame(uchar* data, int width, int height);
protected:
	virtual void keyPressEvent(QKeyEvent* event);
	virtual void keyReleaseEvent(QKeyEvent* event);
private:
	TrobotQt* mainWindow;
	Ui::cameraWindow ui;
};

#endif //CAMERA_WINDOW_H
