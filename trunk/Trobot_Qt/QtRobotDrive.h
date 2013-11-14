#ifndef QT_ROBOT_DRIVE
#define QT_ROBOT_DRIVE

//#define DRIVE_DBG

#include <QtCore/QObject>
#include <string>
#include "../Robot/Robot.h"
#include "../Debug/Debug.h"
#include "ui_trobotqt.h"

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
	QtRobotDrive(Ui::TrobotQtClass* iui, Robot* irobot, Debug* idebug);
	~QtRobotDrive();
	Action getState();
	bool isOpen();
private:
	Ui::TrobotQtClass* ui;
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
};

#endif //QT_ROBOT_DRIVE
