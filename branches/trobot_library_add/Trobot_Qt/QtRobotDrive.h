#ifndef QT_ROBOT_DRIVE
#define QT_ROBOT_DRIVE

//#define DRIVE_DBG

#include <QtCore/QObject>
#include <string>
#include "../Trobot/include/RobotDrive.h"
#include "ui_trobotqt.h"

#define LEFT_CHANNEL 2
#define RIGHT_CHANNEL 1

enum Action {
	Nothing,
	UserDefined,
	Forward,
	Backward,
	Left,
	Right
};

class QtRobotDrive : public QObject
#ifndef DRIVE_DBG
	,public trobot::RobotDrive
#endif
{

	Q_OBJECT

public:
	QtRobotDrive(const std::string& device, Ui::TrobotQtClass* iui, unsigned int baud = 115200);
	~QtRobotDrive();
	Action getState();
private:
	Ui::TrobotQtClass* ui;
	void setButtonsEnabled(bool state);

	Action driveState;
	const int speed;
	int motorVal[2];
public slots:
	void goForward();
	void goBackward();
	void goLeft();
	void goRight();
	void leftMotorStop();
	void rightMotorStop();
	void stop();
	void motorValChanged(int val);
};

#endif //QT_ROBOT_DRIVE
