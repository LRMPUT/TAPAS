#include "../include/rangeSensor.h"

namespace trobot {
	rangeSensor::rangeSensor(unsigned int pinNo, Controller * controller, SensorModel model)
	{
		pinNo_ = pinNo;
		controller_ = controller;
		model_ = model;
	}

	rangeSensor::~rangeSensor(void)
	{
	}

	unsigned int rangeSensor::getDistance() {
		unsigned int dist = 0;
		switch(model_) {
			case GP2Y0A02YK0F: 
				{
					unsigned int raw = controller_->getAnalogReading(pinNo_);
					float volts = raw * controller_->voltsPerUnit;
					dist = (int)(60.495 * pow((float)volts, (float)-1.1904));
					if(dist > 150) //invalid reading
						dist = 99999;

					break;
				}
			case MOBOT_US:
				{
					unsigned int raw = controller_->getAnalogReading(pinNo_);
					if(pinNo_ == 6) {
						dist = raw / 10;
					}
					else {
						float volts = raw * controller_->voltsPerUnit;
						dist = (int)(volts * 100);
					}
					if(dist > 350)
						dist = 99999; //invalid reading

					break;

				}
				
		}
		return dist;

	}

}
