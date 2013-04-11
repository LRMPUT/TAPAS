#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <string>
#include <boost/circular_buffer.hpp>
#include "../include/SerialPort.h"

namespace trobot {
	enum pinModeType {
		modeInput = false,
		modeOutput = true,
	};
	class Controller
	{
	public:
		//Controller(void);
		Controller(unsigned int baud = 9600, const std::string& device = "COM20");
	public:
		~Controller(void);
		//bool			newFramePresent(void);	///< check if there is a new frame of data waiting
		void			processFrame(void);		///< converts frame to readable format
		void			sendData(void);			///< sends data to the controller
		
		void			setPinMode(unsigned int pinNo, pinModeType pinMode);	///< configures digital I/O pin as either output or input
		pinModeType		getPinMode(unsigned int pinNo);							///< gets current mode of a pin
		void			setPinState(unsigned int pinNo, bool state);			///< sets digital I/O pin state high or low; pin must be firs configured as output
		bool			getPinState(unsigned int pinNo);						///< reads the pin state; pin must be firs configured as input
		
		unsigned int	getAnalogReading(unsigned int pinNo); ///< reads analog pin value

		const float		voltsPerUnit;	///< volts per unit of raw value

		//void			connect(void);	///< connects to the physical device
	private:
		int					checkByte(const boost::circular_buffer<char> &data, unsigned char *dest, int i); ///< chcecks for special chars in data, and writes value read into dest, returns index of next byte to check
		void				writeInt16(int data);							///< writes 2 byte value to the serial port
		void				writePinMode(pinModeType * mode, int length);	///< writes array of pinMode values to serial port
		void				writeBoolArray(bool * bools, int length);		///< gues what it does?	
		unsigned int		hex2int(char *hex);	///< converts ascii hex number into integer

		void				searchForDevice();	///< searches for a device and connects to it
		bool				connectionTestOk(); ///< tests if the connection to the controller was properly established
		SerialPort *		serialPort_;		///< pointer to serial port


		static const int	nAnalogPins_ = 7;	///< number of analog inputs in the controller
		static const int	nDigitalPins_ = 14; ///< number of digital I/Os in the controller

		static const int	frameLength = 36;	///< length of the frame
		unsigned int		baud_;				///< COM baud rate
		std::string			device_;			

		unsigned int		analogReading_[ nAnalogPins_ ];
		bool				pinState_[ nDigitalPins_ ];
		pinModeType			pinMode_[ nDigitalPins_ ];

		bool				pinStateToWrite_[ nDigitalPins_ ];
		pinModeType			pinModeToWrite_[ nDigitalPins_ ];

		
		static const char	startChar = '#';
		static const char	stopChar = '$';

		


		

	};
}

#endif //CONTROLLER_H

