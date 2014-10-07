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
