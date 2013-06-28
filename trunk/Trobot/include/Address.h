#ifndef ADDRESS_H
#define ADDRESS_H

namespace trobot 
{
	//! Addresses of all registers.
	enum Address
	{
		//CONFIGURATION REGISTER
		//! Address of communication register.
		COMMUNICATION = 0x00,
		//! Address of miscellaneous configuration register.
		MISC_CONFIG,

		//! Address of X component of magnetic field reference vector register.
		MAG_REF_X,
		//! Address of Y component of magnetic field reference vector register.
		MAG_REF_Y,
		//! Address of Z component of magnetic field reference vector register.
		MAG_REF_Z,

		//! Address of X component of accelerometer reference vector register.
		ACCEL_REF_X,
		//! Address of Y component of accelerometer reference vector register.
		ACCEL_REF_Y,
		//! Address of Z component of accelerometer reference vector register.
		ACCEL_REF_Z,

		//! Address of EKF magnetometer variance register.
		EKF_MAG_VARIANCE,
		//! Address of EKF accelerometer variance register.
		EKF_ACCEL_VARIANCE,
		//! Address of EKF process variance register.
		EKF_PROCESS_VARIANCE,

		//! Address of XY component of gyro bias register.
		GYRO_BIAS_XY,
		//! Address of Z component of gyro bias register.
		GYRO_BIAS_Z,
		//! Address of XY component of accelerometer bias register.
		ACCEL_BIAS_XY,
		//! Address of Z component of accelerometer bias register.
		ACCEL_BIAS_Z,
		//! Address of XY component of magnetometer bias register.
		MAG_BIAS_XY, 
		//! Address of Z component of magnetometer bias register.
		MAG_BIAS_Z, 

		//! Address of 1 accelerometer calibration register.
		ACCEL_CAL_00, 
		//! Address of 2 accelerometer calibration register.
		ACCEL_CAL_01,
		//! Address of 3 accelerometer calibration register.
		ACCEL_CAL_02, 
		//! Address of 4 accelerometer calibration register.
		ACCEL_CAL_10,
		//! Address of 5 accelerometer calibration register.
		ACCEL_CAL_11, 
		//! Address of 6 accelerometer calibration register.
		ACCEL_CAL_12, 
		//! Address of 7 accelerometer calibration register.
		ACCEL_CAL_20, 
		//! Address of 8 accelerometer calibration register.
		ACCEL_CAL_21, 
		//! Address of 9 accelerometer calibration register.
		ACCEL_CAL_22, 

		//! Address of 1 gyro calibration register.
		GYRO_CAL_00, 
		//! Address of 2 gyro calibration register.
		GYRO_CAL_01, 
		//! Address of 3 gyro calibration register.
		GYRO_CAL_02, 
		//! Address of 4 gyro calibration register.
		GYRO_CAL_10, 
		//! Address of 5 gyro calibration register.
		GYRO_CAL_11, 
		//! Address of 6 gyro calibration register.
		GYRO_CAL_12, 
		//! Address of 7 gyro calibration register.
		GYRO_CAL_20, 
		//! Address of 8 gyro calibration register.
		GYRO_CAL_21, 
		//! Address of 9 gyro calibration register.
		GYRO_CAL_22, 

		//! Address of 1 magnetometer calibration register.
		MAG_CAL_00, 
		//! Address of 2 magnetometer calibration register.
		MAG_CAL_01, 
		//! Address of 3 magnetometer calibration register.
		MAG_CAL_02, 
		//! Address of 4 magnetometer calibration register.
		MAG_CAL_10, 
		//! Address of 5 magnetometer calibration register.
		MAG_CAL_11, 
		//! Address of 6 magnetometer calibration register.
		MAG_CAL_12, 
		//! Address of 7 magnetometer calibration register.
		MAG_CAL_20, 
		//! Address of 8 magnetometer calibration register.
		MAG_CAL_21, 
		//! Address of 9 magnetometer calibration register.
		MAG_CAL_22, 

		//DATA REGISTER

		//! Address of status register.
		STATUS = 0x55,

		//! Address of raw data from the X and Y axis rate gyros register.
		GYRO_RAW_XY,
		//! Address of raw data from the Z axis rate gyro register.
		GYRO_RAW_Z,
		//! Address of raw data from the X and Y axis accelerometers register.
		ACCEL_RAW_XY,
		//! Address of raw data from the Z axis accelerometer register.
		ACCEL_RAW_Z,
		//! Address of raw data from the X and Y axis magnetometer register.
		MAG_RAW_XY,
		//! Address of raw data from the Z axis magnetometer register.
		MAG_RAW_Z,

		//! Address of processed data from the X and Y axis rate gyros register.
		GYRO_PROC_XY,
		//! Address of processed data from the Z axis rate gyro register.
		GYRO_PROC_Z,
		//! Address of processed data from the X and Y axis accelerometers register.
		ACCEL_PROC_XY,
		//! Address of processed data from the Z axis accelerometer register.
		ACCEL_PROC_Z,
		//! Address of processed data from the X and Y axis magnetometer register.
		MAG_PROC_XY,
		//! Address of processed data from the Z axis magnetometer register.
		MAG_PROC_Z,

		//! Address of estimated roll and pitch angles register.
		EULER_PHI_THETA,
		//! Address of estimated pitch angle register.
		EULER_PSI,
		//! Address of register which stores the first two components of the estimated quaternion.
		QUAT_AB,
		//! Address of register which stores the third and fourth components of the estimated quaternion.
		QUAT_CD,

		//! Address of 1 covariance matrix register.
		ERROR_COV_00,
		//! Address of 2 covariance matrix register.
		ERROR_COV_01,
		//! Address of 3 covariance matrix register.
		ERROR_COV_02,
		//! Address of 4 covariance matrix register.
		ERROR_COV_03,
		//! Address of 5 covariance matrix register.
		ERROR_COV_10,
		//! Address of 6 covariance matrix register.
		ERROR_COV_11,
		//! Address of 7 covariance matrix register.
		ERROR_COV_12,
		//! Address of 8 covariance matrix register.
		ERROR_COV_13,
		//! Address of 9 covariance matrix register.
		ERROR_COV_20,
		//! Address of 10 covariance matrix register.
		ERROR_COV_21,
		//! Address of 11 covariance matrix register.
		ERROR_COV_22,
		//! Address of 12 covariance matrix register.
		ERROR_COV_23,
		//! Address of 13 covariance matrix register.
		ERROR_COV_30,
		//! Address of 14 covariance matrix register.
		ERROR_COV_31,
		//! Address of 15 covariance matrix register.
		ERROR_COV_32,
		//! Address of 16 covariance matrix register.
		ERROR_COV_33,

		//COMMAND REGISTERS
		//! Address of report the firmware revision register.
		GET_FW_VERSION = 0xAA,
		//! Address of register which write all current configuration values to flash
		FLASH_COMMIT,
		//! Address of zero gyros command register.
		ZERO_GYROS,
		//! Address of reset estimation algorithm register. 
		RESET_EKF,
		//! Address of register which transmit data from all active channels over the UART.
		GET_DATA,
		//!  Address of register which sets the accelerometer reference vector. 
		SET_ACCEL_REF,
		//! Address of register which sets the magnetometer reference vector.
		SET_MAG_REF,
		//!	Address of register which load factory default settings.
		RESET_TO_FACTORY,
		
		//! Address of bad checksum register.
		BAD_CHECKSUM = 0xFD,
		//! Address of unknown address register.
		UNKNOWN_ADDRESS,
		//! Address of invalib batch size register.
		INVALID_BATCH_SIZE,

	};
}

#endif //ADDRESS_H
