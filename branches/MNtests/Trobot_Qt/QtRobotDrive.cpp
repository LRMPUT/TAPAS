#include "QtRobotDrive.h"

#define NO_SLIDER_VAL 1001
#define LEFT_CHANNEL 1
#define RIGHT_CHANNEL 2

using namespace std;

QtRobotDrive::QtRobotDrive(Robot* irobot, Ui::TrobotQtClass* iui):
ui(iui), robot(irobot), driveState(Nothing), speed(800)
{
	motorVal[0] = 0;
	motorVal[1] = 0;
	QObject::connect(ui->robotDriveLeftMotorSlider, SIGNAL(valueChanged(int)), this, SLOT(motorValChanged(int)));
	QObject::connect(ui->robotDriveRightMotorSlider, SIGNAL(valueChanged(int)), this, SLOT(motorValChanged(int)));
	QObject::connect(ui->robotDriveLeftMotorStopButton, SIGNAL(clicked()), this, SLOT(leftMotorStop()));
	QObject::connect(ui->robotDriveRightMotorStopButton, SIGNAL(clicked()), this, SLOT(rightMotorStop()));
	QObject::connect(ui->robotDriveUpButton, SIGNAL(pressed()), this, SLOT(goForward()));
	QObject::connect(ui->robotDriveDownButton, SIGNAL(pressed()), this, SLOT(goBackward()));
	QObject::connect(ui->robotDriveLeftButton, SIGNAL(pressed()), this, SLOT(goLeft()));
	QObject::connect(ui->robotDriveRightButton, SIGNAL(pressed()), this, SLOT(goRight()));
	QObject::connect(ui->robotDriveUpButton, SIGNAL(released()), this, SLOT(stop()));
	QObject::connect(ui->robotDriveDownButton, SIGNAL(released()), this, SLOT(stop()));
	QObject::connect(ui->robotDriveLeftButton, SIGNAL(released()), this, SLOT(stop()));
	QObject::connect(ui->robotDriveRightButton, SIGNAL(released()), this, SLOT(stop()));
	QObject::connect(ui->robotDriveConnectButton, SIGNAL(clicked()), this, SLOT(openRobotDrive()));
	QObject::connect(ui->robotDriveDisconnectButton, SIGNAL(clicked()), this, SLOT(closeRobotDrive()));
	QObject::connect(ui->robotDriveSearchButton, SIGNAL(clicked()), this, SLOT(searchRobotDrive()));
	setButtonsEnabled(false);
}

QtRobotDrive::~QtRobotDrive(){
	setButtonsEnabled(false);
}


Action QtRobotDrive::getState(){
	return driveState;
}

void QtRobotDrive::setButtonsEnabled(bool state){
	ui->robotDriveLeftMotorSlider->setEnabled(state);
	ui->robotDriveRightMotorSlider->setEnabled(state);
	ui->robotDriveLeftMotorStopButton->setEnabled(state);
	ui->robotDriveRightMotorStopButton->setEnabled(state);
	ui->robotDriveUpButton->setEnabled(state);
	ui->robotDriveDownButton->setEnabled(state);
	ui->robotDriveLeftButton->setEnabled(state);
	ui->robotDriveRightButton->setEnabled(state);
	ui->robotDriveSteeringScrollBar->setEnabled(state);
	ui->robotDriveThrottleScrollBar->setEnabled(state);
}

void QtRobotDrive::goForward(){
	ui->robotDriveLeftMotorSlider->setValue(speed);
	ui->robotDriveRightMotorSlider->setValue(speed);
	//motorValChanged(NO_SLIDER_VAL);
	driveState = Forward;
}

void QtRobotDrive::goBackward(){
	ui->robotDriveLeftMotorSlider->setValue(-speed);
	ui->robotDriveRightMotorSlider->setValue(-speed);
	//motorValChanged(NO_SLIDER_VAL);
	driveState = Backward;
}

void QtRobotDrive::goLeft(){
	ui->robotDriveLeftMotorSlider->setValue(-speed);
	ui->robotDriveRightMotorSlider->setValue(speed);
	//motorValChanged(NO_SLIDER_VAL);
	driveState = Left;
}

void QtRobotDrive::goRight(){
	ui->robotDriveLeftMotorSlider->setValue(speed);
	ui->robotDriveRightMotorSlider->setValue(-speed);
	//motorValChanged(NO_SLIDER_VAL);
	driveState = Right;
}

void QtRobotDrive::leftMotorStop(){
	ui->robotDriveLeftMotorSlider->setValue(0);
	//motorValChanged(NO_SLIDER_VAL);
	if(motorVal[LEFT_CHANNEL - 1] == 0){
		driveState = Nothing;
	}
	else{
		driveState = UserDefined;
	}
}

void QtRobotDrive::rightMotorStop(){
	ui->robotDriveRightMotorSlider->setValue(0);
	//motorValChanged(NO_SLIDER_VAL);
	if(motorVal[RIGHT_CHANNEL - 1] == 0){
		driveState = Nothing;
	}
	else{
		driveState = UserDefined;
	}
}

void QtRobotDrive::stop(){
	ui->robotDriveLeftMotorSlider->setValue(0);
	ui->robotDriveRightMotorSlider->setValue(0);
	//motorValChanged(NO_SLIDER_VAL);
	driveState = Nothing;
}

void QtRobotDrive::motorValChanged(int val){
	//cout << "motorValChanged" << endl;
	if(val != NO_SLIDER_VAL){
		if(motorVal[LEFT_CHANNEL - 1] == 0 && motorVal[RIGHT_CHANNEL - 1] == 0){
			driveState = Nothing;
		}
		else{
			driveState = UserDefined;
		}
	}
	motorVal[LEFT_CHANNEL - 1] = ui->robotDriveLeftMotorSlider->value();
	motorVal[RIGHT_CHANNEL - 1] = ui->robotDriveRightMotorSlider->value();
	ui->robotDriveLeftMotorLabel->setText(QString("%1").arg(motorVal[LEFT_CHANNEL - 1]));
	ui->robotDriveRightMotorLabel->setText(QString("%1").arg(motorVal[RIGHT_CHANNEL - 1]));
#ifdef DRIVE_DBG
	printf("robot->setMotorsVel(%d, %d)\n", motorVal[LEFT_CHANNEL - 1], motorVal[RIGHT_CHANNEL - 1]);
#else
	robot->setMotorsVel(motorVal[LEFT_CHANNEL - 1], motorVal[RIGHT_CHANNEL - 1]);
#endif
}

void QtRobotDrive::throttleChanged(int val){

}

void QtRobotDrive::steeringChanged(int val){

}

bool QtRobotDrive::isOpen(){
	return robot->isRobotsDriveOpen();
}

void QtRobotDrive::openRobotDrive(){
	if(ui->robotDrivePortCombo->count() != 0){
		robot->openRobotsDrive(string(ui->robotDrivePortCombo->currentText().toAscii().data()));
		setButtonsEnabled(true);
	}
}

void QtRobotDrive::closeRobotDrive(){
	setButtonsEnabled(false);
	robot->closeRobotsDrive();
}


void QtRobotDrive::searchRobotDrive(){

}

