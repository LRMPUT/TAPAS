/*
` * IMU_driver.cpp
 *
 *  Created on: Sep 1, 2014
 *      Author: smi
 */

#include "../include/IMU_driver.h"
#include<cmath>
#include<chrono>

// Global pointer to instance of IMUDriver
IMU_driver* pointerToIMUDriver;

void filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size,
		u8 callback_type) {
	mip_field_header *field_header;
	u8 *field_data;
	u16 field_offset = 0;
	switch (callback_type) {
	case MIP_INTERFACE_CALLBACK_VALID_PACKET: {
		mip_filter_attitude_euler_angles curr_filter_angles;

		///
		//Loop through all of the data fields
		///
		while (mip_get_next_field(packet, &field_header, &field_data,
				&field_offset) == MIP_OK) {
			///
			// Decode the field
			///
			switch (field_header->descriptor) {
			///
			// Estimated Attitude, Euler Angles
			///
			case MIP_FILTER_DATA_ATT_EULER_ANGLES: {
				memcpy(&curr_filter_angles, field_data,
						sizeof(mip_filter_attitude_euler_angles));

				//For little-endian targets, byteswap the data field
				mip_filter_attitude_euler_angles_byteswap(&curr_filter_angles);

#ifdef IMU_DRIVER_DEBUG
				printf("IMU: Current angle measures : %f %f %f\n",
						curr_filter_angles.pitch, curr_filter_angles.roll,
						curr_filter_angles.yaw);
#endif
				pointerToIMUDriver->setEuler(curr_filter_angles.roll,
						curr_filter_angles.pitch, curr_filter_angles.yaw);

				pointerToIMUDriver->setTimestamp(
						std::chrono::high_resolution_clock::now());

			}
				break;

			default:
				break;
			}
		}

	}
		break;
	case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR: {
		printf("Checksum error\n");
	}
		break;
	case MIP_INTERFACE_CALLBACK_TIMEOUT: {
		printf("Timeout error\n");
	}
		break;
	default:
		break;
	}
}

void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size,
		u8 callback_type) {
	mip_field_header *field_header;
	u8 *field_data;
	u16 field_offset = 0;
	switch (callback_type) {
	case MIP_INTERFACE_CALLBACK_VALID_PACKET: {
		mip_ahrs_scaled_gyro curr_ahrs_gyro;
		mip_ahrs_scaled_accel curr_ahrs_accel;
		mip_ahrs_scaled_mag curr_ahrs_mag;

		while (mip_get_next_field(packet, &field_header, &field_data,
				&field_offset) == MIP_OK) {

			///
			// Decode the field
			///
			switch (field_header->descriptor) {

			case MIP_AHRS_DATA_ACCEL_SCALED: {
				memcpy(&curr_ahrs_accel, field_data,
						sizeof(mip_ahrs_scaled_accel));

				//For little-endian targets, byteswap the data field
				mip_ahrs_scaled_accel_byteswap(&curr_ahrs_accel);

				pointerToIMUDriver->setAccel(curr_ahrs_accel.scaled_accel);

#ifdef IMU_DRIVER_DEBUG
				printf("IMU_driver: Acceelerometer data: %f %f %f\n",
						curr_ahrs_accel.scaled_accel[0],
						curr_ahrs_accel.scaled_accel[1],
						curr_ahrs_accel.scaled_accel[2]);
#endif
			}
				break;

				///
				// Scaled Gyro
				///

			case MIP_AHRS_DATA_GYRO_SCALED: {
				memcpy(&curr_ahrs_gyro, field_data,
						sizeof(mip_ahrs_scaled_gyro));

				//For little-endian targets, byteswap the data field
				mip_ahrs_scaled_gyro_byteswap(&curr_ahrs_gyro);

				pointerToIMUDriver->setGyro(curr_ahrs_gyro.scaled_gyro);

#ifdef IMU_DRIVER_DEBUG
				printf("IMU_driver: Gyro data: %f %f %f\n",
						curr_ahrs_gyro.scaled_gyro[0],
						curr_ahrs_gyro.scaled_gyro[1],
						curr_ahrs_gyro.scaled_gyro[2]);
#endif
			}
				break;

				///
				// Scaled Magnetometer
				///

			case MIP_AHRS_DATA_MAG_SCALED: {
				memcpy(&curr_ahrs_mag, field_data, sizeof(mip_ahrs_scaled_mag));

				//For little-endian targets, byteswap the data field
				mip_ahrs_scaled_mag_byteswap(&curr_ahrs_mag);

				pointerToIMUDriver->setMag(curr_ahrs_mag.scaled_mag);

#ifdef IMU_DRIVER_DEBUG
				printf("IMU_driver: Mag data: %f %f %f\n",
						curr_ahrs_mag.scaled_mag[0],
						curr_ahrs_mag.scaled_mag[1],
						curr_ahrs_mag.scaled_mag[2]);
#endif
			}
				break;

			default:
				break;

			}
		}

	}
		break;
	case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR: {
		printf("Checksum error\n");
	}
		break;
	case MIP_INTERFACE_CALLBACK_TIMEOUT: {
		printf("Timeout error\n");
	}
		break;
	default:
		break;
	}
}

IMU_driver::IMU_driver() : accValid(false), gyroValid(false), magValid(false), eulerValid(false) {
	pointerToIMUDriver = this;
	runThread = false;
}

IMU_driver::~IMU_driver() {
	printf("~IMU_driver\n");
	runThread = false;
	if(processingThread.joinable()){
		processingThread.join();
	}
	mip_sdk_port_close(&device_interface.port_handle);
	pointerToIMUDriver = NULL;
	printf("End ~IMU_driver\n");
}

void IMU_driver::updateIMU() {
	printf("IMU: Started update thread --> will provide the data\n");
	while (runThread) {
		//Update the parser
		mip_interface_update(&device_interface);
		//Be nice to other programs
		usleep(10);
	}
	printf("IMU: Finished update thread --> no more new data\n");
}

void IMU_driver::openPort(std::string& device) {
	char deviceName[100];
	strcpy(deviceName, device.c_str());
	if (mip_interface_init(deviceName, 230400, &device_interface,
	DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK)
		printf("IMU: Not connected to the IMU\n");

	u8 com_mode = MIP_SDK_GX4_25_IMU_STANDARD_MODE;

	///
	//Set communication mode
	///

#ifdef IMU_DRIVER_DEBUG
	printf("IMU: Attempting to set communications mode to IMU Standard mode\n");
#endif

	while (mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_WRITE,
			&com_mode) != MIP_INTERFACE_OK) {
	}

	///
	//Verify device mode setting
	///

	while (mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_READ,
			&com_mode) != MIP_INTERFACE_OK) {
	}

	if (com_mode == MIP_SDK_GX4_25_IMU_STANDARD_MODE) {

#ifdef IMU_DRIVER_DEBUG
		printf("IMU: Communications mode IMU Standard.\n");
#endif
	}

	printf(
			"IMU: Performing Gyro Bias capture.\nIMU: Please keep device stationary "
					"during the 5 second gyro bias capture interval\n");

	u16 duration = 5000; //milliseconds
	float bias_vector[3] = { 0 };

	while (mip_3dm_cmd_capture_gyro_bias(&device_interface, duration,
			bias_vector) != MIP_INTERFACE_OK) {
	}

#ifdef IMU_DRIVER_DEBUG
	printf(
			"IMU: Gyro Bias Captured:\nbias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n",
			bias_vector[0], bias_vector[1], bias_vector[2]);

	printf("IMU: Set Gyro Bias Vector\n");
#endif

	while (mip_3dm_cmd_gyro_bias(&device_interface, MIP_FUNCTION_SELECTOR_WRITE,
			bias_vector) != MIP_INTERFACE_OK) {
	}

	// Checking if coning and sculling compensation is set
#ifdef IMU_DRIVER_DEBUG
	printf("Reading Coning and Sculling compensation enabled state:\n");
#endif
	u8 enable = 1;

	while (mip_3dm_cmd_coning_sculling_compensation(&device_interface,
	MIP_FUNCTION_SELECTOR_READ, &enable) != MIP_INTERFACE_OK) {
	}

	if (enable == MIP_3DM_CONING_AND_SCULLING_DISABLE)
		printf("IMU: CONING_AND_SCULLING is DISABLED !!!\n");

#ifdef IMU_DRIVER_DEBUG
	printf("IMU: Getting the Estimation Filter datastream base rate\n");
	u16 base_rate;
	while (mip_3dm_cmd_get_filter_base_rate(&device_interface, &base_rate)
			!= MIP_INTERFACE_OK) {
	}

	printf("IMU: FILTER Base Rate => %d Hz\n", base_rate);
#endif

	// Setting up the callback
	if (mip_interface_add_descriptor_set_callback(&device_interface,
	MIP_FILTER_DATA_SET, NULL, &filter_packet_callback) != MIP_INTERFACE_OK)
		printf("IMU: Something went wrong at adding the callback\n");

	if (mip_interface_add_descriptor_set_callback(&device_interface,
	MIP_AHRS_DATA_SET, NULL, &ahrs_packet_callback) != MIP_INTERFACE_OK)
		printf("IMU: Something went wrong at adding the callback\n");

	//	Setup the AHRS datastream format
#ifdef IMU_DRIVER_DEBUG
	printf("IMU:  Setting the AHRS message format\n");
#endif

	u8 data_stream_format_descriptors[10];
	data_stream_format_descriptors[0] = MIP_AHRS_DATA_ACCEL_SCALED;
	data_stream_format_descriptors[1] = MIP_AHRS_DATA_GYRO_SCALED;
	data_stream_format_descriptors[2] = MIP_AHRS_DATA_MAG_SCALED;

	u16 data_stream_format_decimation[10];
	data_stream_format_decimation[0] = 0x05;
	data_stream_format_decimation[1] = 0x32;
	data_stream_format_decimation[2] = 0x32;

	u8 data_stream_format_num_entries = 3;

	while (mip_3dm_cmd_ahrs_message_format(&device_interface,
	MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries,
			data_stream_format_descriptors, data_stream_format_decimation)
			!= MIP_INTERFACE_OK) {
	}

	//  Setup the FILTER datastream format
#ifdef IMU_DRIVER_DEBUG
	printf("IMU: Setting the Estimation Filter datastream format\n");
#endif

	data_stream_format_num_entries = 1;
	data_stream_format_descriptors[0] = MIP_FILTER_DATA_ATT_EULER_ANGLES;
	data_stream_format_descriptors[1] = 0x00;
	data_stream_format_descriptors[2] = 0x00;

	data_stream_format_decimation[0] = 0x0a;
	data_stream_format_decimation[1] = 0x00;
	data_stream_format_decimation[2] = 0x00;

	while (mip_3dm_cmd_filter_message_format(&device_interface,
	MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries,
			data_stream_format_descriptors, data_stream_format_decimation)
			!= MIP_INTERFACE_OK) {
	}

#ifdef IMU_DRIVER_DEBUG
	printf("IMU: Estimation Filter set ok!\n");
#endif

	// Starting the streams
	while (mip_3dm_cmd_continuous_data_stream(&device_interface,
	MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_AHRS_DATASTREAM, &enable)
			!= MIP_INTERFACE_OK) {
	}

	while (mip_3dm_cmd_continuous_data_stream(&device_interface,
	MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_INS_DATASTREAM, &enable)
			!= MIP_INTERFACE_OK) {
	}

	// Starting the another thread to update the imu values
	runThread = true;
	processingThread = std::thread(&IMU_driver::updateIMU, this);
}

bool IMU_driver::isDataValid(){
	return (accValid && gyroValid && magValid && eulerValid);
}

//Read data from gyro.
void IMU_driver::getGyro(float* gyroValues) {
	std::unique_lock < std::mutex > lckGyro(gyroMtx);
	for (int i = 0; i < 3; i++) {
		gyroValues[i] = gyro[i];
	}
	lckGyro.unlock();
}

void IMU_driver::setGyro(float * _gyro) {
	std::unique_lock < std::mutex > lckGyro(gyroMtx);
	for (int i = 0; i < 3; i++){
		gyro[i] = _gyro[i];
	}
	lckGyro.unlock();
	gyroValid = true;
}

//Read data from accelerometer.
void IMU_driver::getAccel(float *accValues) {
	std::unique_lock < std::mutex > lckAcc(accMtx);
	for (int i = 0; i < 3; i++) {
		accValues[i] = acc[i];
	}
	lckAcc.unlock();
}

void IMU_driver::setAccel(float * _accel) {
	static std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();
	static bool printed = false;

	std::unique_lock < std::mutex > lckAcc(accMtx);
	for (int i = 0; i < 3; i++){
		acc[i] = _accel[i];
	}
	lckAcc.unlock();

	std::unique_lock < std::mutex > lckHistoryAcc(accHistoryMtx);
	accHistory.push_back( sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]) );

	if ( accHistory.size() > 200 )
	{
		if(printed == false){
			printf("Imu accHistory time diff = %d ms\n",
					std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count());
			printed = true;
		}
		accHistory.pop_front();
	}
	lckHistoryAcc.unlock();

	accValid = true;
}

//Read data from magnetometer.
void IMU_driver::getMag(float* magValues) {
	std::unique_lock < std::mutex > lckMag(magMtx);
	for (int i = 0; i < 3; i++) {
		magValues[i] = mag[i];
	}
	lckMag.unlock();
}

void IMU_driver::setMag(float * _mag) {
	std::unique_lock < std::mutex > lckMag(magMtx);
	for (int i = 0; i < 3; i++){
		mag[i] = _mag[i];
	}
	lckMag.unlock();
	magValid = true;
}

//Read Euler angles.
void IMU_driver::getEuler(float *eulerValues) {
	std::unique_lock < std::mutex > lckEuler(eulerMtx);
	for (int i = 0; i < 3; i++) {
		eulerValues[i] = euler[i];
	}
	lckEuler.unlock();
}

void IMU_driver::setEuler(float roll, float pitch, float yaw) {
	std::unique_lock < std::mutex > lckEuler(eulerMtx);
	euler[0] = roll*180/M_PI;
	euler[1] = pitch*180/M_PI;
	euler[2] = yaw*180/M_PI;
	lckEuler.unlock();
	eulerValid = true;
}


float IMU_driver::getAccVariance()
{
	std::unique_lock < std::mutex > lckHistoryAcc(accHistoryMtx);

	float mean = 0;
	for (std::list<float>::iterator it = accHistory.begin();it!=accHistory.end();++it)
		mean += *it;
	mean /= accHistory.size();

	float variance = 0;
	for (std::list<float>::iterator it = accHistory.begin();it!=accHistory.end();++it)
			variance += pow(*it- mean, 2);
	variance /= (accHistory.size() - 1);

	lckHistoryAcc.unlock();

	return variance;
}

// get Timestamp of last orientation update
std::chrono::high_resolution_clock::time_point IMU_driver::getTimestamp() {
	return timestamp;
}
void IMU_driver::setTimestamp(
		std::chrono::high_resolution_clock::time_point _timestamp) {
	timestamp = _timestamp;
}

// print data
void IMU_driver::printInformation() {
	base_device_info_field device_info;
	u8 temp_string[20] = { 0 };

	printf("IMU: Getting Device Information\n");\
	while (mip_base_cmd_get_device_info(&device_interface, &device_info)
			!= MIP_INTERFACE_OK) {
	}

	printf("\n\nDevice Info:\n");
	printf("---------------------------------------------\n");

	memcpy(temp_string, device_info.model_name,
	BASE_DEVICE_INFO_PARAM_LENGTH * 2);
	printf("Model Name       => %s\n", temp_string);

	memcpy(temp_string, device_info.model_number,
	BASE_DEVICE_INFO_PARAM_LENGTH * 2);
	printf("Model Number     => %s\n", temp_string);

	memcpy(temp_string, device_info.serial_number,
	BASE_DEVICE_INFO_PARAM_LENGTH * 2);
	printf("Serial Number    => %s\n", temp_string);

	memcpy(temp_string, device_info.lotnumber,
	BASE_DEVICE_INFO_PARAM_LENGTH * 2);
	printf("Lot Number       => %s\n", temp_string);

	memcpy(temp_string, device_info.device_options,
	BASE_DEVICE_INFO_PARAM_LENGTH * 2);
	printf("Options          => %s\n", temp_string);

	printf("Firmware Version => %d.%d.%.2d\n\n",
			(device_info.firmware_version) / 1000,
			(device_info.firmware_version) % 1000 / 100,
			(device_info.firmware_version) % 100);

	printf("\n\n");
}

void IMU_driver::pingDevice() {
	// Try to ping the GX4-25
	printf("IMU: Pinging Device\n");

	while (mip_base_cmd_ping(&device_interface) != MIP_INTERFACE_OK) {
	}

	printf("IMU: Device responded\n");
}

