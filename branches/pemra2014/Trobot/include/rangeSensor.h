#ifndef RANGE_SENSOR_H
#define RANGE_SENSOR_H

#include "Controller.h"

namespace trobot {
	enum SensorModel {
		GP2Y0A02YK0F,	///< IR sensor by SHARP, range 20 - 150 cm; 
		MOBOT_US		///< ultrasonic sensor by MOBOT, range 3 - 350 cm;
	};
	class rangeSensor
	{
	public:
		rangeSensor(unsigned int pinNo, Controller * controller, SensorModel model);
		unsigned int		getDistance(); ///< returns distance in centimeters, 99999 when invalid reading
	public:
		~rangeSensor(void);
	private:
		Controller *		controller_;
		unsigned int		pinNo_;		
		SensorModel			model_;
		

	};
}

#endif //RANGE_SENSOR_H

