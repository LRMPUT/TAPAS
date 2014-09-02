/*
 * IMU_driver.h
 *
 *  Created on: Sep 1, 2014
 *      Author: smi
 */

#ifndef IMU_DRIVER_H_
#define IMU_DRIVER_H_


#include <ctime>
#include <string>
#include <chrono>

// Microstrain dependencies
extern "C" {
	#include "mip_sdk.h"
	#include "byteswap_utilities.h"
	#include "mip_gx4_imu.h"
	#include "mip_gx4_25.h"
}
#include <stdio.h>
#include <unistd.h>

#define DEFAULT_PACKET_TIMEOUT_MS  1000

#define MIP_SDK_GX4_25_IMU_STANDARD_MODE  0x01
#define MIP_SDK_GX4_25_IMU_DIRECT_MODE	  0x02

class IMU_driver {
	mip_interface device_interface;
//public:
//	//! Serial port baud length
	static const int Baud = 115200;
//
//	//Write new miscellaneous configuration to UM6.
//	eMsgFromUM6 WriteMiscellaneous();
//	//Read miscellaneous configuration from UM6.
//	eMsgFromUM6 ReadMiscellaneous();
//
//	//Read data from gyro.
//	XYZ_Response GetGyro();
//	//Read data from accelerometer.
//	XYZ_Response GetAccel();
//	//Read data from magnetometer.
//	XYZ_Response GetMag();
//	//Read Euler angles.
//	Euler_Response GetEuler();
//
//	//Enable broadcasting mode
//	void EnableBroadcasting(bool accel, bool gyro, bool mag, bool euler);
//	//Disable broadcasting mode
//	void DisableBroadcasting();
//
//	int8 testConnection();
//
//	//Change serial port configuration.
//	void ChangeSerialPortConfiguartion(eBaud baud, const std::string& device);
//
//	// get Timestamp of last orientation update
//	std::chrono::high_resolution_clock::time_point getTimestamp();
//
public:
	IMU_driver();

	void openPort(std::string& device);
//
//private:
//	bool broadcasting_;	///< variable that indicates whether IMU is in broadcasting mode or not
//
//	std::string device_;
//	void searchForDevice();	///< searches for a device and connects to it
//	bool connectionTestOk(); ///< tests if the connection to the controller was properly established
//
//	boost::thread * thread_;
//	volatile bool threadEnd;
//
//	SerialPort* serial_port;
//
//	// Timestamp of last measurement
//	std::chrono::high_resolution_clock::time_point timestamp;
//
//	//! Make array with all register.
//	void ConfigArray();
//	//! Add register to SERIAL PAKIET STRUCTURE.
//	void addRegisterToSend(Packet & packet, _array rejestr[], Address A,
//			uint8 BL);
//	//!  Write data to array "data".
//	void addDataToRegister(Packet & packet, uint8 A, uint8 BL);
//	//! Write new data to Register.
//	void UpdateRegister(uint8 rejestr[]);
//	//! Send packet to UM6.
//	void sendPacket(Packet packet, uint8 BL = 0);
//	//! Get answer from UM6.
//	void Get_Data_From_UM6();
};

#endif /* IMU_DRIVER_H_ */
