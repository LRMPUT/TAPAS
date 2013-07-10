#include <QtGui/QPainter>
#include <iostream>
#include "ImuChart.h"

using namespace std;

#define REFRESH_RATE 10
#define IMU_DATA_RATE 100

#define BAUD 115200

#define MIN_SCALE 0.001
#define MAX_SCALE 1000
#define SCALE_FACT 100000
#define SCALE_FACT_POW 5

Value values[NUM_VALUES] = {
	Value(4*trobot::ACCEL_PROC_XY + 2,	Qt::red,		"accel x",		0.000183105),
	Value(4*trobot::ACCEL_PROC_XY,		Qt::darkRed,	"accel y",		0.000183105),
	Value(4*trobot::ACCEL_PROC_Z + 2,	Qt::green,		"accel z",		0.000183105),
	Value(4*trobot::GYRO_PROC_XY + 2,	Qt::darkGreen,	"gyro x",		0.0610352),
	Value(4*trobot::GYRO_PROC_XY,		Qt::blue,		"gyro y",		0.0610352),
	Value(4*trobot::GYRO_PROC_Z + 2,	Qt::darkBlue,	"gyro z",		0.0610352),
	Value(4*trobot::MAG_PROC_XY + 2,	Qt::cyan,		"mag x",		0.000305176),
	Value(4*trobot::MAG_PROC_XY,		Qt::darkCyan,	"max y",		0.000305176),
	Value(4*trobot::MAG_PROC_Z + 2,		Qt::magenta,	"mag z",		0.000305176),
	Value(4*trobot::EULER_PHI_THETA + 2,Qt::darkMagenta,"euler roll",	0.0109863),
	Value(4*trobot::EULER_PHI_THETA,	Qt::yellow,		"euler pitch",	0.0109863),
	Value(4*trobot::EULER_PSI,			Qt::darkYellow,	"euler yaw",	0.0109863)
};

ImuChart::ImuChart(Ui::TrobotQtClass* iui) : chart(NULL), imu(NULL), ui(iui) {
	cout << "Constructing ImuChart" << endl;
	timerRefresh.setInterval(1000/REFRESH_RATE);
	timerCollectData.setInterval(1000/IMU_DATA_RATE);

	//ui->imuTimeScrollBar->setValue(-10);
	//ui->imuValueScrollBar->setValue(-240);

	QObject::connect(ui->imuConnectButton, SIGNAL(clicked()), this, SLOT(connect()));
	QObject::connect(ui->imuDisconnectButton, SIGNAL(clicked()), this, SLOT(disconnect()));
	QObject::connect(ui->imuStartButton, SIGNAL(clicked()), this, SLOT(start()));
	QObject::connect(ui->imuStopButton, SIGNAL(clicked()), this, SLOT(stop()));
	QObject::connect(ui->imuClearButton, SIGNAL(clicked()), this, SLOT(clear()));
	QObject::connect(&timerRefresh, SIGNAL(timeout()), this, SLOT(update()));
	QObject::connect(&timerCollectData, SIGNAL(timeout()), this, SLOT(collectData()));
	QObject::connect(ui->imuTimeScrollBar, SIGNAL(valueChanged(int)), this, SLOT(repaint()));
	QObject::connect(ui->imuValueScrollBar, SIGNAL(valueChanged(int)), this, SLOT(repaint()));
	QObject::connect(ui->accelScaleLineEdit, SIGNAL(editingFinished()), this, SLOT(repaint()));
	QObject::connect(ui->gyroScaleLineEdit, SIGNAL(editingFinished()), this, SLOT(repaint()));
	QObject::connect(ui->magScaleLineEdit, SIGNAL(editingFinished()), this, SLOT(repaint()));
	QObject::connect(ui->eulerScaleLineEdit, SIGNAL(editingFinished()), this, SLOT(repaint()));
	QObject::connect(ui->imuTimeScaleLineEdit, SIGNAL(editingFinished()), this, SLOT(repaint()));

	values[0].checkBox = ui->accelXcheckBox;
	values[0].scale = ui->accelScaleLineEdit;
	values[1].checkBox = ui->accelYcheckBox;
	values[1].scale = ui->accelScaleLineEdit;
	values[2].checkBox = ui->accelZcheckBox;
	values[2].scale = ui->accelScaleLineEdit;

	values[3].checkBox = ui->gyroXcheckBox;
	values[3].scale = ui->gyroScaleLineEdit;
	values[4].checkBox = ui->gyroYcheckBox;
	values[4].scale = ui->gyroScaleLineEdit;
	values[5].checkBox = ui->gyroZcheckBox;
	values[5].scale = ui->gyroScaleLineEdit;

	values[6].checkBox = ui->magXcheckBox;
	values[6].scale = ui->magScaleLineEdit;	
	values[7].checkBox = ui->magYcheckBox;
	values[7].scale = ui->magScaleLineEdit;	
	values[8].checkBox = ui->magZcheckBox;
	values[8].scale = ui->magScaleLineEdit;

	values[9].checkBox = ui->eulerRollCheckBox;
	values[9].scale = ui->eulerScaleLineEdit;		
	values[10].checkBox = ui->eulerPitchCheckBox;
	values[10].scale = ui->eulerScaleLineEdit;		
	values[11].checkBox = ui->eulerYawCheckBox;
	values[11].scale = ui->eulerScaleLineEdit;

	setEnableChart(false);
	ui->imuStartButton->setEnabled(false);
	ui->imuStopButton->setEnabled(false);
	ui->imuClearButton->setEnabled(false);
}

ImuChart::~ImuChart(){
	if(chart != NULL){
		delete chart;
	}
	if(imu != NULL){
		delete imu;
	}
}

void ImuChart::createChart(){
	if(imu == NULL){
		return;
	}
	int count = 0;
	for(int i = 0; i < NUM_VALUES; i++){
		if(values[i].checkBox->isChecked() == true){
			count++;
		}
	}
	if(chart != NULL){
		delete chart;
	}
	chart = new Chart(count, IMU_DATA_RATE);
	ui->imuDisplayLabel->setPixmap(*chart);
	//scales.resize(count);
	ui->imuTimeScrollBar->setMinimum(0);
	ui->imuTimeScrollBar->setMaximum(CHART_RES_X);
	ui->imuTimeScrollBar->setValue(-10);
	ui->imuValueScrollBar->setMinimum(CHART_RES_Y/2 - 500);
	ui->imuValueScrollBar->setMaximum(CHART_RES_Y/2 + 500);
	ui->imuValueScrollBar->setValue(-CHART_RES_Y/2);
	int ind = 0;
	for(int i = 0; i < count; i++){
		if(values[i].checkBox->isChecked() == true){
			chart->setColor(ind++, values[i].color);
		}
	}
	compScales();
	compOrig();
	setChartScalesAndOrig();
	setEnableChart(true);
}

void ImuChart::drawAxes(){
	const int timeOffset = 15;
	const int valOffset = 15;
	//horizontal
	drawAxis(scaleT, timeOffset, valOffset, Qt::black, "s", 0);
	//vertical
	for(int i = 0; i < 4; i++){
		drawAxis(scales[4*i], valOffset, (i + 1)*timeOffset, values[4*i].color, "", 1);
	}
}

//0 - horizontal, 1 - vertical

void ImuChart::drawAxis(float scale, int offsetBeg, int offsetAxis, QColor color, QString unit, int orient){
	int res = (orient == 0 ? CHART_RES_X : CHART_RES_Y);
	int orig = (orient == 0 ? origX : origY);
	float begVal = (offsetBeg - orig)*scale;
	float endVal = (res - offsetBeg - orig)*scale;
	int stepMult = (endVal - begVal)*SCALE_FACT;
	int stepPow = -SCALE_FACT_POW - 1;
	while(stepMult >= 10){
		stepMult /= 10;
		stepPow++;
	}
	if(2 <= stepMult && stepMult <=4){
		stepMult = 5;
	}
	if(stepMult >= 6){
		stepMult = 1;
		stepPow += 10;
	}
	float step = stepMult*pow((float)10, stepPow);
	float scalePos = ((int)(begVal/step) + 1)*step;
	QPainter painter(chart);
	painter.setPen(color);
	painter.setFont(QFont("Candara", 7));
	if(orig == 0){
		painter.drawLine(offsetBeg, offsetAxis, res, offsetAxis);
	}
	else{
		painter.drawLine(offsetAxis, offsetBeg, offsetAxis, res);
	}
	while(scalePos < endVal){
		int scalePix = round(scalePos/scale) + orig;
		if(orig == 0){
			painter.drawLine(scalePix, offsetAxis, scalePix, offsetAxis + 5);
			painter.drawText(scalePix, offsetAxis - 10, QString("%1").arg(scalePos, 0, 'g', 2));
		}
		else{
			painter.drawLine(offsetAxis, scalePix, offsetAxis + 5, scalePix);
			painter.drawText(offsetAxis - 10, scalePix, QString("%1").arg(scalePos, 0, 'g', 2));
		}
		scalePos += step;
	}
	//units
	painter.end();
}

void ImuChart::setEnableChart(bool val){
	ui->imuValueScrollBar->setEnabled(val);
	ui->imuTimeScrollBar->setEnabled(val);
	ui->accelXcheckBox->setEnabled(!val);
	ui->accelYcheckBox->setEnabled(!val);
	ui->accelZcheckBox->setEnabled(!val);
	ui->gyroXcheckBox->setEnabled(!val);
	ui->gyroYcheckBox->setEnabled(!val);
	ui->gyroZcheckBox->setEnabled(!val);
	ui->magXcheckBox->setEnabled(!val);
	ui->magYcheckBox->setEnabled(!val);
	ui->magZcheckBox->setEnabled(!val);
	ui->eulerRollCheckBox->setEnabled(!val);
	ui->eulerPitchCheckBox->setEnabled(!val);
	ui->eulerYawCheckBox->setEnabled(!val);
}

void ImuChart::setEnableStartButton(bool val){
	ui->imuStartButton->setEnabled(val);
	ui->imuStopButton->setEnabled(!val);
}

void ImuChart::compScales(){
	scaleT = ui->imuTimeScaleLineEdit->text().toFloat()/CHART_RES_X;
	for(int i = 0; i < NUM_VALUES; i++){
		scales[i] = values[i].scale->text().toFloat()/CHART_RES_Y;
	}
}

void ImuChart::compOrig(){
	origX = -ui->imuTimeScrollBar->value();
	origY = -ui->imuValueScrollBar->value();
}

void ImuChart::setChartScalesAndOrig(){
	cout << "Setting orig = (" << origX << ", " << origY << ")" << endl;
	cout << "Setting time scale = " << scaleT << endl;
	chart->setOrigin(QPoint(origX, origY));
	chart->setTimeScale(scaleT);
	int ind = 0;
	for(int i = 0; i < NUM_VALUES; i++){
		if(values[i].checkBox->isChecked() == true){
			chart->setScale(ind++, scales[i]);
		}
	}
}

int ImuChart::round(float a){
	return (int)(a + 0.5);
}

void ImuChart::start(){
	setEnableStartButton(false);
	if(chart == NULL){
		createChart();
		cout << "Chart created" << endl;
	}
	timerRefresh.start();
	timerCollectData.start();
}

void ImuChart::stop(){
	setEnableStartButton(true);
	timerRefresh.stop();
	timerCollectData.stop();
}

void ImuChart::clear(){
	//setEnableStartButton(false);
	stop();
	delete chart;
	chart = NULL;
}

void ImuChart::collectData(){
	int ind = 0;
	vector<vector<float> > newData;
	for(int i = 0; i < NUM_VALUES; i++){
		if(values[i].checkBox->isChecked() == true){
			float tmp = (short)((imu->Register[values[i].address / 4]._int >> 8*(values[i].address % 4)) & 0xffff);
			cout << "Extracted value " << (short)((imu->Register[values[i].address / 4]._int >> 8*(values[i].address % 4)) & 0xffff) << endl;
			newData.push_back(vector<float>(1, tmp));
		}
	}
	//imu->fetch = true;
	//cout << "Adding data to chart, vector size = " << newData.size() << endl;
	chart->addData(newData);
}

void ImuChart::update(){
	chart->updateFromLast();
	ui->imuDisplayLabel->setPixmap(*chart);
	//cout << "Label repainted" << endl;
}

void ImuChart::repaint(){
	if(chart == NULL){
		return;
	}
	compScales();
	compOrig();
	setChartScalesAndOrig();
	chart->repaint();
	ui->imuDisplayLabel->setPixmap(*chart);
}

void ImuChart::connect(){
	cout << "Connecting" << endl;
	imu = new trobot::Imu(BAUD, ui->imuPortCombo->currentText().toAscii().data());
	setEnableStartButton(true);
	ui->imuClearButton->setEnabled(true);
	cout << "Connected" << endl;
}

void ImuChart::disconnect(){
	clear();
	delete imu;
	setEnableStartButton(true);
	ui->imuClearButton->setEnabled(true);
}

std::vector<double> ImuChart::getImuData()
{
	trobot::XYZ_Response tmp;
	trobot::Euler_Response tmp2;
	std::vector<double> ret;
	// gyro
	tmp = imu->GetGyro();
	ret.push_back(tmp.x); ret.push_back(tmp.y); ret.push_back(tmp.z);
	// accel
	tmp = imu->GetAccel();
	ret.push_back(tmp.x); ret.push_back(tmp.y); ret.push_back(tmp.z);
	// magneto
	tmp = imu->GetMag();
	ret.push_back(tmp.x); ret.push_back(tmp.y); ret.push_back(tmp.z);
	// euler
	tmp2 = imu->GetEuler();
	ret.push_back(tmp2.phi); ret.push_back(tmp2.psi); ret.push_back(tmp2.theta);
}
