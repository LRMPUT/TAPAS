/*
 * IMU_driver.cpp
 *
 *  Created on: Sep 1, 2014
 *      Author: smi
 */

#include "../include/IMU_driver.h"

int x = 0;



void filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size,
		u8 callback_type) {
	mip_field_header *field_header;
	u8 *field_data;
	u16 field_offset = 0;
	switch (callback_type) {
	case MIP_INTERFACE_CALLBACK_VALID_PACKET: {
		printf("Received correct packet from IMU\n");
		x++;
		mip_filter_attitude_euler_angles curr_filter_angles;

		///
		//Loop through all of the data fields
		///

		while (mip_get_next_field(packet, &field_header, &field_data,
				&field_offset) == MIP_OK) {

			///
			// Decode the field
			///
			printf("Filed header type : %d\n", field_header->descriptor);
			switch (field_header->descriptor) {
			///
			// Estimated Attitude, Euler Angles
			///

			case MIP_FILTER_DATA_ATT_EULER_ANGLES: {
				memcpy(&curr_filter_angles, field_data, sizeof(mip_filter_attitude_euler_angles));

				//For little-endian targets, byteswap the data field
				mip_filter_attitude_euler_angles_byteswap(&curr_filter_angles);

				printf("Current angle measures : %f %f %f\n",
						curr_filter_angles.pitch, curr_filter_angles.roll,
						curr_filter_angles.yaw);

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

IMU_driver::IMU_driver() {

}

void IMU_driver::openPort(std::string& device) {
	char deviceName[100];
	strcpy(deviceName, device.c_str());
	if (mip_interface_init(deviceName, 115200, &device_interface,
	DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK)
		printf("IMU: Not connected to the IMU\n");

	u8 com_mode = 0;

	com_mode = MIP_SDK_GX4_25_IMU_STANDARD_MODE;

	///
	//Set communication mode
	///

	printf("IMU: Attempting to set communications mode to IMU Direct mode\n");

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

		printf("IMU: Communications mode IMU Standard.\n");

		usleep(1500);
	}

	printf("IMU: Getting the Estimation Filter datastream base rate\n");

	u16 base_rate;
	while (mip_3dm_cmd_get_filter_base_rate(&device_interface, &base_rate)
			!= MIP_INTERFACE_OK) {
	}

	printf("IMU: FILTER Base Rate => %d Hz\n", base_rate);

	usleep(1500);


	// Setting up the callback
	if (mip_interface_add_descriptor_set_callback(&device_interface,
	MIP_FILTER_DATA_SET, NULL, &filter_packet_callback) != MIP_INTERFACE_OK)
		printf("IMU: Something went wrong at adding the callback\n");


	///
	//Setup the FILTER datastream format
	///
	printf("IMU: Setting the Estimation Filter datastream format\n");
	u8 data_stream_format_descriptors[10];
	u16 data_stream_format_decimation[10];
	u8 data_stream_format_num_entries = 0;
	data_stream_format_descriptors[0] = MIP_FILTER_DATA_ATT_EULER_ANGLES;

	data_stream_format_decimation[0] = 0x32;

	data_stream_format_num_entries = 1;

	while (mip_3dm_cmd_filter_message_format(&device_interface,
	MIP_FUNCTION_SELECTOR_WRITE, &data_stream_format_num_entries,
			data_stream_format_descriptors, data_stream_format_decimation)
			!= MIP_INTERFACE_OK) {
	}
	printf("IMU: Estimation Filter set ok!\n");

	// Starting the stream
	u8 enable = 1;

//	while(mip_3dm_cmd_continuous_data_stream(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_AHRS_DATASTREAM, &enable) != MIP_INTERFACE_OK){}

	while (mip_3dm_cmd_continuous_data_stream(&device_interface,
	MIP_FUNCTION_SELECTOR_WRITE, MIP_3DM_INS_DATASTREAM, &enable)
			!= MIP_INTERFACE_OK) {
	}

//	if (mip_interface_add_descriptor_set_callback(&device_interface,
//	MIP_AHRS_DATA_SET, NULL, &ahrs_packet_callback) != MIP_INTERFACE_OK)
//		return -1;

	printf("Started!\n");
	while (1) {
		//Update the parser
		mip_interface_update(&device_interface);
		//Be nice to other programs
		usleep(10);
	}
}
