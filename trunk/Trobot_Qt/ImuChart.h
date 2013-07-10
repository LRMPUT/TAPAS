#ifndef IMU_CHART_H
#define IMU_CHART_H

#include <QtCore/QTimer>
#include <QtCore/QString>
#include <QtGui/QColor>
#include <vector>
#include "ui_trobotqt.h"
#include "Chart.h"
#include "../Trobot/include/Imu.h"
#include "../Trobot/include/Address.h"

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
	trobot::Imu* imu;
	Ui::TrobotQtClass* ui;
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
	ImuChart(Ui::TrobotQtClass* iui);
	~ImuChart();

	// Just for dumping
	std::vector<double> getImuData();
	bool testConnection();
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
