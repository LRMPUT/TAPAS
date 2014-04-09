#ifndef IMU_H
#define IMU_H

#include "../include/SerialPort.h"
#include "../include/Address.h"
#include "../include/const_inttypes.h"
#include <ctime>
#include <string>

namespace trobot 
{
	//! Baud Rate
	enum eBaud
	{
		_BAUD_9600 = 0,	
		_BAUD_14400,	
		_BAUD_19200,	
		_BAUD_38400,	
		_BAUD_57600,	
		_BAUD_115200,
	};

	//!Answer from UM6.
	enum eMsgFromUM6
	{
		_NOTHING = 0,
		_WRITE_COMPLETE,			
		_READ_COMPLETE,			
		_COMMAND_COMPLETE,		
		_COMMAND_FAILED,		
		_BAD_CHEKSUM,			
		_UNKNOWN_ADDRESS,			
		_INVALID_BATCH_SIZE,	
	};

	enum eMsgReadState
	{
		COMPLETE,
		HEADER,
		LENGTH,
	};

	typedef struct
	{
		int8 byte1;
		int8 byte2;
		int8 byte3;
		int8 byte4;
	} float_bytes;

	typedef union
	{
		float flt;
		float_bytes bytes;
	} _float2bytes;

	typedef struct
	{
		int8 byte1;
		int8 byte2;
		int8 byte3;
		int8 byte4;
	} uint_bytes;

	typedef union
	{
		uint32					uint;
		uint_bytes				bytes;
	} uint_32;
		
	typedef union
	{
		uint32					_int;
		_float2bytes			_float;
	}_array;


	//---------------- CONFIGURATION REGISTER ---------------------------------
	//---------------- CommunicationRegister ---------------------------------
	
	//!Struct is responsible for first byte of COMMUNICATION REGISTER.
	typedef struct
	{
		//!Bits are in charge of baud rate. Variable is composed of three bits.
		/**Used to set sensor serial port baud rate. 
		*/
		uint8 baud_rate			: 3 ;	// bits  0, 1, 2	
		//!These bits are reserved. Variable is composed of five bits.  
		uint8 RES				: 5 ;	// bit   3,4,5,6,7						
	} sBitsB1_Communication;

	//!Union is responsible for first byte of COMMUNICATION REGISTER.
	typedef union
	{
		//!Variable is in charge of bits representation on first byte of COMMUNICATION REGISTER.
		sBitsB1_Communication	bitsB1;
		//!Variable is in charge of byte representation on first byte of COMMUNICATION REGISTER.
		uint8					sByteB1_Communication;
	} ByteB1_Communication;

	//!Struct is responsible for second byte of COMMUNICATION REGISTER.
	typedef struct
	{	
		//!These bits are reserved. Variable is composed of five bits. 
		uint8 RES				: 5 ;	// bit   0,1,2,3,4
		//!Covariance matrix transmission enabled. Variable is composed of one bit. 
		uint8 COV				: 1 ;	// bit   5
		//!Euler angle transmission enabled. Variable is composed of one bit. 
		uint8 EU				: 1 ;	// bit   6
		//!Quaternion transmission enabled. Variable is composed of one bit. 
		uint8 QT				: 1 ;	// bit   7
	} sBitsB2_Communication;

	//!Union is responsible for second byte of COMMUNICATION REGISTER.
	typedef union
	{
		//!Variable is in charge of bits representation on second byte of COMMUNICATION REGISTER.
		sBitsB2_Communication	bitsB2;
		//!Variable is in charge of byte representation on second byte of COMMUNICATION REGISTER.
		uint8					sByteB2_Communication;
	} ByteB2_Communication;

	//!Struct is responsible for third byte of COMMUNICATION REGISTER.
	typedef struct 
	{
		//!Processed magnetometer data transmission enabled. Variable is composed of one bit. 
		uint8 MP				: 1 ;	// bits  0	
		//!Processed accelerometer data transmission enabled. Variable is composed of one bit. 
		uint8 AP				: 1 ;	// bits  1	
		//!Processed gyro data transmission enabled. Variable is composed of one bit. 
		uint8 GP				: 1 ;	// bits  2	
		//!Raw magnetometer data transmission enabled. Variable is composed of one bit. 
		uint8 MR				: 1 ;	// bits  3
		//!Raw accelerometer data transmission enabled. Variable is composed of one bit. 
		uint8 AR				: 1 ;	// bits  4
		//!Raw gyro data transmission enabled. Variable is composed of one bit. 
		uint8 GR				: 1 ;	// bits  5
		//!Broadcast mode enabled. Variable is composed of one bit. 
		uint8 BEN				: 1 ;	// bits  6	
		//!!This bit is reserved. Variable is composed of one bit. 
		uint8 RES				: 1 ;	// bits  7	
	} sBitsB3_Communication;

	//!Union is responsible for third byte of COMMUNICATION REGISTER.
	typedef union
	{
		//!Variable is in charge of bits representation on third byte of COMMUNICATION REGISTER.
		sBitsB3_Communication	bitsB3;
		//!Variable is in charge of byte representation on third byte of COMMUNICATION REGISTER.
		uint8					sByteB3_Communication;
	} ByteB3_Communication;

	//!Struct is responsible for bytes representation of COMMUNICATION REGISTER.
	typedef struct
	{
		//!It is null byte of communication register. 	
		/** These bits specify how often a data packets are automatically transmitted over the serial port 
		*when broadcast mode is enabled.  
		*/
		uint8					Byte0;
		//! It is first byte of communication register. 
		ByteB1_Communication	Byte1;
		//! It is second byte of communication register. 
		ByteB2_Communication	Byte2;
		//! It is third byte of communication register. 
		ByteB3_Communication	Byte3;
	} sCommunicationRegister;

	//!Union is responsible for COMMUNICATION REGISTER (Communication configuration settings). 
	typedef union
	{
		//!Variable is in charge of bytes representation of COMMUNICATION REGISTER.
		sCommunicationRegister  Bytes;
		//!Variable is in charge of all COMMUNICATION REGISTER.
		uint32					Communication;
	} CommunicationRegister;
	//---------------- END: CommunicationRegister ---------------------------------

	//---------------- MiscellaneousConfiguration ---------------------------------
	//!Struct is responsible for third byte of MISCELLANEOUS CONFIGURATION register.
	typedef struct
	{
		//!These bits are reserved. Variable is composed of three bits.
		uint8 RS				: 3 ;	// bits  0,1,2	
		//!Specifies whether PPS timing is enabled. This will be implemented in future firmware revisions. Variable is composed of one bit. 
		uint8 PPS				: 1 ;	// bits  3	
		//!Specifies whether quaternion state estimation is enabled. Variable is composed of one bit.
		uint8 QUAT				: 1 ;	// bits  4	
		//!Enables startup gyro calibration.  If this bit is set, then gyros will be calibrated automatically on sensor startup. Variable is composed of one bit.  
		uint8 CAL				: 1 ;	// bits  5	
		//!EKF accelerometer updates enabled (pitch and roll angle correction). Variable is composed of one bit. 
		uint8 AUE				: 1 ;	// bits  6
		//!EKF magnetometer updates enabled (yaw angle correction). Variable is composed of one bit.
		uint8 MUE				: 1 ;	// bits  7		
	} sBitsB3_MISC_Config;

	//!Union is responsible for third byte of MISCELLANEOUS CONFIGURATION register.
	typedef union
	{
		//!Variable is in charge of bits representation on third byte of MISCELLANEOUS CONFIGURATION register.
		sBitsB3_MISC_Config		bitsB3;
		//!Variable is in charge of byte representation on third byte of MISCELLANEOUS CONFIGURATION register.
		uint8					sByteB3_MISC_Config;
	} ByteB3_MISC_Config;

	//!Struct is responsible for bytes representation of MISCELLANEOUS CONFIGURATION register.
	typedef struct
	{
		//! It is null byte of MISCELLANEOUS CONFIGURATION register. These byte is reserved. 
		uint8					ByteB0;
		//! It is first byte of MISCELLANEOUS CONFIGURATION register. These byte is reserved. 
		uint8					ByteB1;
		//! It is second byte of MISCELLANEOUS CONFIGURATION register. These byte is reserved. 
		uint8					ByteB2;
		//! It is third byte of MISCELLANEOUS CONFIGURATION register. 
		ByteB3_MISC_Config		ByteB3;
	} sMiscellaneousRegister;

	//!Union is responsible for MISCELLANEOUS CONFIGURATION register (Miscellaneous configuration options). 
	typedef union
	{
		//!Variable is in charge of bytes representation of MISCELLANEOUS CONFIGURATION register.
		sMiscellaneousRegister	Bytes;
		//!Variable is in charge of all MISCELLANEOUS CONFIGURATION register.
		uint32					Miscellaneous;
	} MiscellaneousConfigurationRegister;
	//---------------- END: MiscellaneousConfiguration ---------------------------------

	//---------------- GYRO BIAS XY ---------------------------------
	//!Struct is responsible for particulal members of GYRO BIAS XY register.
	typedef struct
	{
		//!Variable is in charge of Y gyro bias (16-bit, little-endian, 2's complement).
		int16					Gyro_Bias_Y;
		//!Variable is in charge of X gyro bias (16-bit, little-endian, 2's complement).
		int16					Gyro_Bias_X;
	} Gyro_XY;

	//!Union is responsible for GYRO BIAS XY register. 
	/** Stores the values used to compensate for rate gyro bias on the X and Y gyro axes.  
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of GYRO BIAS XY register.
		Gyro_XY					gyro_XY;
		//!Variable is in charge of GYRO BIAS XY register.
		uint32					Gyro_Bias_XY;
	} Gyro_Bias_XY_Register;
	//---------------- END: GYRO BIAS XY ---------------------------------

	//---------------- GYRO BIAS Z ---------------------------------
	//!Struct is responsible for particulal members of GYRO BIAS Z register.
	typedef struct
	{
		//!Variable is reserved.
		int16					RES;
		//!Variable is in charge of Z gyro bias (16-bit, little-endian, 2's complement).
		int16					Gyro_Bias_Z;
	} Gyro_Z;

	//!Union is responsible for GYRO BIAS Z register. 
	/**Stores the values used to compensate for rate gyro bias on the gyro Z axis.  
	* The bias value is stored as a 16-bit 2's complement signed integer. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of GYRO BIAS Z register.
		Gyro_Z					gyro_Z;
		//!Variable is in charge of GYRO BIAS Z register.
		uint32					Gyro_Bias_Z;
	} Gyro_Bias_Z_Register;
	//---------------- END: GYRO BIAS Z ---------------------------------

	//---------------- ACCEL BIAS XY ---------------------------------
	//!Struct is responsible for particulal members of ACCEL BIAS XY register.
	typedef struct
	{
		//!Variable is in charge of Y accel bias (16-bit, little-endian, 2's complement).
		int16					Accel_Bias_Y;
		//!Variable is in charge of X accel bias (16-bit, little-endian, 2's complement).
		int16					Accel_Bias_X;
	} Accel_XY;
	
	//!Union is responsible for ACCEL BIAS XY register. 
	/**Stores the values used to compensate for bias on the X and Y accelerometer axes.  
	*The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of ACCEL BIAS XY register.
		Accel_XY				accel_XY;
		//!Variable is in charge of ACCEL BIAS XY" register.
		uint32					Accel_Bias_XY;
	} Accel_Bias_XY_Register;
	//---------------- END: ACCEL BIAS XY ---------------------------------

	//---------------- ACCEL BIAS Z ---------------------------------
	//!Struct is responsible for particulal members of ACCEL BIAS Z register.
	typedef struct
	{
		//!Variable is reserved.
		int16					RES;
		//!Variable is in charge of Z accel bias (16-bit, little-endian, 2's complement).
		int16					Accel_Bias_Z;
	} Accel_Z;

	//!Union is responsible for ACCEL BIAS Z register. 
	/**Stores the values used to compensate for bias on the accelerometer Z axis.  
	*The bias value is stored as a 16-bit 2's complement signed integer. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of ACCEL BIAS Z register.
		Accel_Z					accel_Z;
		//!Variable is in charge of ACCEL BIAS Z register.
		uint32					Accel_Bias_Z;
	} Accel_Bias_Z_Register;
	//---------------- END: ACCEL BIAS Z ---------------------------------

	//---------------- MAG BIAS XY ---------------------------------
	//!Struct is responsible for particulal members of MAG BIAS XY register.
	typedef struct
	{
		//!Variable is in charge of Y mag bias (16-bit, little-endian, 2's complement).
		int16					Mag_Bias_Y;
		//!Variable is in charge of X mag bias (16-bit, little-endian, 2's complement).
		int16					Mag_Bias_X;
	} Mag_XY;

	//!Union is responsible for MAG BIAS XY register. 
	/**Stores the values used to compensate for bias on the X and Y magnetometer axes.  
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of MAG BIAS XY register.
		Mag_XY					mag_XY;
		//!Variable is in charge of MAG BIAS XY register.
		uint32					Mag_Bias_XY;
	} Mag_Bias_XY_Register;
	//---------------- END: MAG BIAS XY ---------------------------------

	//---------------- MAG BIAS Z ---------------------------------
	//!Struct is responsible for particulal members of MAG BIAS Z register.
	typedef struct
	{
		//!Variable is reserved.
		int16					RES;
		//!Variable is in charge of Z mag bias (16-bit, little-endian, 2's complement).
		int16					Mag_Bias_Z;
	} Mag_Z;
	
	//!Union is responsible for MAG BIAS Z register. 
	/**Stores the values used to compensate for bias on the Z magnetometer axes.  
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of MAG BIAS Z register.
		Mag_Z					mag_Z;
		//!Variable is in charge of MAG BIAS Z register.
		uint32					Mag_Bias_Z;
	} Mag_Bias_Z_Register;
	//---------------- END: MAG BIAS Z ---------------------------------
	//---------------- END: CONFIGURATION REGISTER ---------------------------------

	//---------------- DATA REGISTER ---------------------------------
	//---------------- STATUS REGISTER ---------------------------------
	//!Struct is responsible for null byte of STATUS REGISTER.
	typedef struct
	{
		//!If asserted, indicates that a self-test operation was completed. Variable is composed of one bit. 
		uint8 ST				: 1 ;	// bits  0, 1, 2
		//!These bits are reserved. Variable is composed of five bits. 
		uint8 RES				: 7 ;	// bit   3,4,5,6,7						
	} sBitsB0_Status;

	//!Union is responsible for null byte of STATUS REGISTER.
	typedef union
	{
		//!Variable is in charge of bits representation on null byte of STATUS REGISTER.
		sBitsB0_Status			bitsB0;
		//!Variable is in charge of byte representation on null byte of STATUS REGISTER.
		uint8					sByteB0_Status;
	} ByteB0_Status;

	//!Struct is responsible for first byte ofSTATUS REGISTER.
	typedef struct
	{	
		//!These bits are reserved. Variable is composed of five bits. 
		uint8 RES				: 5 ;	// bit   0,1,2,3,4
		//!Indicates that the processor did not receive data from the magnetic sensor for longer than expected. Variable is composed of one bit. 
		uint8 MAG_DEL			: 1 ;	// bit   5
		//!Indicates that the processor did not receive data from the accelerometer for longer than expected. Variable is composed of one bit. 
		uint8 ACC_DEL			: 1 ;	// bit   6
		//!Indicates that the processor did not receive data from the gyros for longer than expected. Variable is composed of one bit. 
		uint8 GYR_DEL			: 1 ;	// bit   7
	} sBitsB1_Status;

	//!Union is responsible for first byte of STATUS REGISTER.
	typedef union
	{
		//!Variable is in charge of bits representation on first byte of STATUS REGISTER.
		sBitsB1_Status			bitsB1;
		//!Variable is in charge of byte representation on first byte of STATUS REGISTER.
		uint8					sByteB1_Status;
	} ByteB1_Status;

	//!Struct is responsible for second byte of STATUS REGISTER.
	typedef struct
	{
		//!Indicates that the EKF state estimates became divergent and the EKF was forced to restart. Variable is composed of one bit. 
		uint8 EKF_DIV			: 1 ;	// bits  0		
		//!Indicates that there was a bus error while communicating with the magnetic sensor. Variable is composed of one bit. 
		uint8 BUS_MAG			: 1 ;	// bits  1	
		//!Indicates that there was a bus error while communicating with the accelerometer. Variable is composed of one bit. 
		uint8 BUS_ACC			: 1 ;	// bits  2	
		//!Indicates that there was a bus error while communicating with the gyros. Variable is composed of one bit. 
		uint8 BUS_GYR			: 1 ;	// bits  3	
		//!Indicates that the self-test operation failed on the magnetometer z-axis. Variable is composed of one bit. 
		uint8 ST_MZ				: 1 ;	// bits  4	
		//!Indicates that the self-test operation failed on the magnetometer y-axis. Variable is composed of one bit. 
		uint8 ST_MY				: 1 ;	// bits  5	
		//!Indicates that the self-test operation failed on the magnetometer x-axis. Variable is composed of one bit. 
		uint8 ST_MX				: 1 ;	// bits  6	
		//!Indicates that the self-test operation failed on the accelerometer z-axis. Variable is composed of one bit. 
		uint8 ST_AZ				: 1 ;	// bits  7	
	} sBitsB2_Status;

	//!Union is responsible for second byte of STATUS REGISTER.
	typedef union
	{
		//!Variable is in charge of bits representation on second byte of STATUS REGISTER.
		sBitsB2_Status			bitsB2;
		//!Variable is in charge of byte representation on second byte of STATUS REGISTER.
		uint8					sByteB2_Status;
	} ByteB2_Status;

	//!Struct is responsible for third byte of STATUS REGISTER.
	typedef struct
	{
		//!Indicates that the self-test operation failed on the accelerometer y-axis. Variable is composed of one bit. 
		uint8 ST_AY				: 1 ;	// bits  0		
		//!Indicates that the self-test operation failed on the accelerometer x-axis. Variable is composed of one bit. 
		uint8 ST_AX				: 1 ;	// bits  1	
		//!Indicates that the self-test operation failed on the rate gyro z-axis. Variable is composed of one bit. 
		uint8 ST_GZ				: 1 ;	// bits  2	
		//!Indicates that the self-test operation failed on the rate gyro y-axis. Variable is composed of one bit. 
		uint8 ST_GY				: 1 ;	// bits  3	
		//!Indicates that the self-test operation failed on the rate gyro x-axis. Variable is composed of one bit. 
		uint8 ST_GX				: 1 ;	// bits  4	
		//!Indicates that rate gyro startup initialization failed. Variable is composed of one bit. 
		uint8 GYR_INI			: 1 ;	// bits  5	
		//!Indicates that rate accelerometer startup initialization failed. Variable is composed of one bit. 
		uint8 ACC_INI			: 1 ;	// bits  6	
		//!Indicates that rate magnetometer startup initialization failed. Variable is composed of one bit. 
		uint8 MAG_INI			: 1 ;	// bits  7	
	} sBitsB3_Status;

	//!Union is responsible for third byte of STATUS REGISTER.
	typedef union
	{
		//!Variable is in charge of bits representation on third byte of STATUS REGISTER.
		sBitsB3_Status			bitsB3;
		//!Variable is in charge of byte representation on third byte of STATUS REGISTER.
		uint8					sByteB3_Status;
	} ByteB3_Status;

	//!Struct is responsible for bytes representation of STATUS REGISTER.
	typedef struct
	{
		//!It is null byte of STATUS REGISTER. 
		ByteB0_Status			Byte0;
		//!It is first byte of STATUS REGISTER. 
		ByteB1_Status			Byte1;
		//!It is second byte of STATUS REGISTER. 
		ByteB2_Status			Byte2;
		//!It is third byte of STATUS REGISTER. 
		ByteB3_Status			Byte3;
	} sStatusRegister;

	//!Union is responsible for STATUS REGISTER. 
	typedef union
	{
		//!Variable is in charge of bytes representation of STATUS REGISTER.
		sStatusRegister			Bytes;
		//!Variable is in charge of all STATUS REGISTER.
		uint32					Status;
	} StatusRegister;
	//---------------- END: STATUS REGISTER ---------------------------------

	//---------------- GYRO RAW XY ---------------------------------
	//!Struct is responsible for particulal members of GYRO RAW XY register.
	typedef struct
	{
		//!Variable is in charge of Y gyro raw (16-bit, little-endian, 2's complement).
		int16					G_Raw_Y;
		//!Variable is in charge of X gyro raw (16-bit, little-endian, 2's complement).
		int16					G_Raw_X;
	} Gyro_R_XY;

	//!Union is responsible for GYRO RAW XY register. 
	/** Stores the most recent data acquired from the X and Y rate gyro axes.  
	* Data is the data measured directly by the sensor.
	* The raw sensor data for each axis is stored as a 16-bit 2's complement integer. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of GYRO RAW XY register.
		Gyro_R_XY				gyro_r_xy;
		//!Variable is in charge of gyro GYRO RAW XY register.
		int						Gyro_Raw_XY;
	}Gyro_Raw_XY_Register;
	//---------------- END: GYRO RAW XY ---------------------------------

	//---------------- GYRO RAW Z ---------------------------------
	//!Struct is responsible for particulal members of GYRO RAW Z register.
	typedef struct
	{
		//!Variable is reserved.
		int16					RES;
		//!Variable is in charge of Z gyro raw (16-bit, little-endian, 2's complement)..
		int16					G_Raw_Z;
	} Gyro_R_Z;

	//!Union is responsible for GYRO RAW Z register. 
	/**  Stores the most recent data acquired from the Z rate gyro axes.  
	* Data is the data measured directly by the sensor.
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of GYRO RAW Z register.
		Gyro_R_Z				gyro_r_z;
		//!Variable is in charge of GYRO RAW Z register.
		uint32					Gyro_Raw_Z;
	} Gyro_Raw_Z_Register;
	//---------------- END: GYRO RAW Z ---------------------------------

	//---------------- ACCEL RAW XY ---------------------------------
	//!Struct is responsible for particulal members of ACCEL RAW XY register.
	typedef struct
	{
		//!Variable is in charge of Y accelerometer raw (16-bit, little-endian, 2's complement).
		int16					A_Raw_Y;
		//!Variable is in charge of X accelerometer raw (16-bit, little-endian, 2's complement).
		int16					A_Raw_X;
	} Accel_R_XY;

	//!Union is responsible for ACCEL RAW XY register. 
	/** Stores the most recent data acquired from the X and Y accelerometer axes.
	* Data is the data measured directly by the sensor.
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of ACCEL RAW XY register.
		Accel_R_XY				accel_r_xy;
		//!Variable is in charge of ACCEL RAW XY register.
		uint32					Accel_Raw_XY;
	}Accel_Raw_XY_Register;
	//---------------- END: ACCEL RAW XY ---------------------------------

	//---------------- ACCEL RAW Z ---------------------------------
	//!Struct is responsible for particulal members of ACCEL RAW Z register.
	typedef struct
	{
		//!Variable is reserved.
		int16					RES;
		//!Variable is in charge of Z accelerometer raw (16-bit, little-endian, 2's complement).
		int16					A_Raw_Z;
	} Accel_R_Z;

	//!Union is responsible for ACCEL RAW Z register. 
	/** Stores the values used to compensate for rate gyro raw on the Z gyro axes.
	* Data is the data measured directly by the sensor.
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of ACCEL RAW Z register.
		Accel_R_Z				accel_r_z;
		//!Variable is in charge of ACCEL RAW Z register.
		uint32					Accel_Raw_Z;
	} Accel_Raw_Z_Register;
	//---------------- END: ACCEL RAW Z ---------------------------------

	//---------------- MAG RAW XY ---------------------------------
	//!Struct is responsible for particulal members of MAG RAW XY register.
	typedef struct
	{
		//!Variable is in charge of Y magnetometer raw (16-bit, little-endian, 2's complement).
		int16					M_Raw_Y;
		//!Variable is in charge of X magnetometer raw (16-bit, little-endian, 2's complement).
		int16					M_Raw_X;
	} Mag_R_XY;

	//!Union is responsible for MAG RAW XY register. 
	/** Stores the most recent data acquired from the X and Y magnetometer axes.
	* Data is the data measured directly by the sensor.
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of MAG RAW XY register.
		Mag_R_XY				mag_r_xy;
		//!Variable is in charge of MAG RAW XY register.
		uint32					Mag_Raw_XY;
	}Mag_Raw_XY_Register;
	//---------------- END: MAG RAW XY ---------------------------------

	//----------------MAG RAW Z ---------------------------------
	//!Struct is responsible for particulal members of MAG RAW Z register.
	typedef struct
	{
		//!Variable is reserved.
		int16					RES;
		//!Variable is in charge of Z magnetometer raw (16-bit, little-endian, 2's complement).
		int16					M_Raw_Z;
	} Mag_R_Z;

	//!Union is responsible for MAG RAW Z register. 
	/** Stores the most recent data acquired from the Z magnetometer axes.
	* Data is the data measured directly by the sensor. 
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of MAG RAW Z register.
		Mag_R_Z					mag_r_z;
		//!Variable is in charge of MAG RAW Z register.
		uint32					Mag_Raw_Z;
	} Mag_Raw_Z_Register;
	//---------------- END: MAG RAW Z ---------------------------------

	//---------------- GYRO PROC XY ---------------------------------
	//!Struct is responsible for particulal members of GYRO PROC XY register.
	typedef struct
	{
		//!Variable is in charge of Y gyro processed (16-bit, little-endian, 2's complement).
		int16					G_Proc_Y;
		//!Variable is in charge of X gyro processed (16-bit, little-endian, 2's complement).
		int16					G_Proc_X;
	} Gyro_P_XY;

	//!Union is responsible for GYRO PROC XY register. 
	/** Stores the most recent processed data acquired from the X and Y axis rate gyros.
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of GYRO PROC XY register.
		Gyro_P_XY				gyro_proc_xy;
		//!Variable is in charge of GYRO PROC XY register.
		uint32					Gyro_Proc_XY;
	}Gyro_Proc_XY_Register;
	//---------------- END: GYRO  PROC XY ---------------------------------

	//---------------- GYRO PROC Z ---------------------------------
	//!Struct is responsible for particulal members of GYRO PROC Z register.
	typedef struct
	{
		//!Variable is reserved.
		int16					RES;
		//!Variable is in charge of Z gyro processed (16-bit, little-endian, 2's complement).
		int16					G_Proc_Z;
	} Gyro_P_Z;

	//!Union is responsible for GYRO PROC Z register. 
	/** Stores the most recent processed data acquired from the Z axis rate gyros.
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of GYRO PROC Z register.
		Gyro_P_Z				gyro_p_z;
		//!Variable is in charge of GYRO PROC Z register.
		uint32					Gyro_Proc_Z;
	} Gyro_Proc_Z_Register;
	//---------------- END: GYRO  PROC Z ---------------------------------

	//---------------- ACCEL PROC XY ---------------------------------
	//!Struct is responsible for particulal members of ACCEL PROC XY register.
	typedef struct
	{
		//!Variable is in charge of Y accelerometer processed (16-bit, little-endian, 2's complement).
		int16					A_Proc_Y;
		//!Variable is in charge of X accelerometer processed (16-bit, little-endian, 2's complement).
		int16					A_Proc_X;
	} Accel_P_XY;

	//!Union is responsible for ACCEL PROC XY register. 
	/** Stores the most recent processed data acquired from the X and Y axis accelerometers.
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of ACCEL PROC XY register.
		Accel_P_XY				accel_p_xy;
		//!Variable is in charge of ACCEL PROC XY register.
		uint32					Accel_Proc_XY;
	}Accel_Proc_XY_Register;
	//---------------- END: ACCEL  PROC XY ---------------------------------

	//---------------- ACCEL PROC Z ---------------------------------
	//!Struct is responsible for particulal members of ACCEL PROC Z register.
	typedef struct
	{
		//!Variable is reserved.
		int16					RES;
		//!Variable is in charge of Z accelerometer processed  (16-bit, little-endian, 2's complement).
		int16					A_Proc_Z;
	} Accel_P_Z;

	//!Union is responsible for ACCEL PROC Z register. 
	/** Stores the most recent processed data acquired from the Z axis accelerometer.
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of ACCEL PROC Z register.
		Accel_P_Z				accel_p_z;
		//!Variable is in charge of ACCEL PROC Z register.
		uint32					Accel_Proc_Z;
	} Accel_Proc_Z_Register;
	//---------------- END: ACCEL PROC Z ---------------------------------

	//---------------- MAG PROC XY ---------------------------------
	//!Struct is responsible for particulal members of MAG PROC XY register.
	typedef struct
	{
		//!Variable is in charge of Y magnetometer processed (16-bit, little-endian, 2's complement).
		int16					M_Proc_Y;
		//!Variable is in charge of X magnetometer processed (16-bit, little-endian, 2's complement).
		int16					M_Proc_X;
	} Mag_P_XY;

	//!Union is responsible for MAG PROC XY register. 
	/** Stores the most recent processed data acquired from the X and Y axis magnetometers.   
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of MAG PROC XY register.
		Mag_P_XY				mag_p_xy;
		//!Variable is in charge of MAG PROC XY register.
		uint32					Mag_Proc_XY;
	}Mag_Proc_XY_Register;
	//---------------- END: MAG PROC XY ---------------------------------

	//----------------MAG PROC Z ---------------------------------
	//!Struct is responsible for particulal members of MAG PROC Z register.
	typedef struct
	{
		//!Variable is reserved.
		int16					RES;
		//!Variable is in charge of Z magnetometer processed (16-bit, little-endian, 2's complement).
		int16					M_Proc_Z;
	} Mag_P_Z;

	//!Union is responsible for MAG PROC Z register. 
	/** Stores the most recent processed data acquired from the Z axis magnetometers.  
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of MAG PROC Z register.
		Mag_P_Z					mag_p_z;
		//!Variable is in charge of MAG PROC Z register.
		uint32					Mag_Proc_Z;
	} Mag_Proc_Z_Register;
	//---------------- END: MAG PROC Z ---------------------------------

	//---------------- EULER PHI THETA ---------------------------------
	//!Struct is responsible for particulal members of EULER PHI THETA register.
	typedef struct
	{
		//!Variable is in charge of Theta(16-bit, little-endian, 2's complement).
		int16					Theta;
		//!Variable is in charge of Phi (16-bit, little-endian, 2's complement).
		int16					Phi;
	} Euler_P_T;

	//!Union is responsible for EULER PHI THETA register. 
	/** Stores the most recently computed roll (phi) and pitch (theta) angle estimates. 
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of EULER PHI THETA register.
		Euler_P_T				euler_p_t;
		//!Variable is in charge of EULER PHI THETA register.
		uint32					Euler_Phi_Theta;
	}Euler_Phi_Theta_Register;
	//---------------- END: EULER PHI THETA ---------------------------------

	//---------------- EULER PSI ---------------------------------
	//!Struct is responsible for particulal members of EULER PSI register.
	typedef struct
	{
		//!Variable is in charge of Y gyro bias (16-bit, little-endian, 2's complement).
		int16					RES;
		//!Variable is in charge of Psi (16-bit, little-endian, 2's complement).
		int16					Psi;
	} Euler_P;

	//!Union is responsible for EULER PSI register. 
	/** Stores the most recently computed yaw (psi) angle estimate.  
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of EULER PSI register.
		Euler_P					euler_p;
		//!Variable is in charge of EULER PSI register.
		uint32					Euler_Psi;
	} Euler_Psi_Register;
	//---------------- END: EULER PSI ---------------------------------

	//---------------- QUAT AB ---------------------------------
	//!Struct is responsible for particulal members of QUAT AB register.
	typedef struct
	{
		//!Variable is in charge of B (Second component of quaternion vector, 16-bit, little-endian, 2's complement).
		int16					B;
		//!Variable is in charge of A (First component of quaternion vector, 16-bit, little-endian, 2's complement).
		int16					A;
	} Quat_A_B;

	//!Union is responsible for QUAT AB register. 
	/** Stores the first two components of the most recent quaternion attitude estimate. 
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of QUAT AB register.
		Quat_A_B				quat_ab;
		//!Variable is in charge of QUAT AB register.
		uint32					Quat_AB;
	}Quat_AB_Register;
	//---------------- END: QUAT AB ---------------------------------

	//---------------- QUAT CD ---------------------------------
	//!Struct is responsible for particulal members of QUAT CD register.
	typedef struct
	{
		//!Variable is in charge of D gyro bias (Second component of quaternion vector, 16-bit, little-endian, 2's complement).
		int16					D;
		//!Variable is in charge of C gyro bias (First component of quaternion vector, 16-bit, little-endian, 2's complement).
		int16					C;
	} Quat_C_D;

	//!Union is responsible for gyro bias QUAT CD register. 
	/** Stores the second two components of the most recent quaternion attitude estimate. 
	* The bias values are stored as 16-bit 2's complement signed integers. 
	*/
	typedef union
	{
		//!Variable is in charge of particulal members of QUAT CD register.
		Quat_C_D				quat_cd;
		//!Variable is in charge of QUAT CD register.
		uint32					Quat_CD;
	}Quat_CD_Register;
	//---------------- END: QUAT CD ---------------------------------
	//---------------- END: DATA REGISTER ---------------------------------

	//!Struct is responsible for bytes representation of GET FW VERSION.
	typedef struct
	{
		//!It is null byte of  GET FW VERSION. 
		char					Byte0;
		//!It is first byte of  GET FW VERSION. 
		char					Byte1;
		//!It is second byte of  GET FW VERSION. 
		char					Byte2;
		//!It is third byte of  GET FW VERSION. 
		char					Byte3;
	} sGetFwVersionRegister;

	//!Union is responsible for GET FW VERSION. 
	typedef union
	{
		//!Variable is in charge of bytes representation of GET FW VERSION.
		sGetFwVersionRegister	Bytes;
		//!Variable is in charge of all GET FW VERSION.
		uint32					FwVersion;
	} GetFwVersionRegister;

	//---------------- Packet Type ---------------------------------
	//!Struct is responsible for bits structure of packet type.
	typedef struct
	{
		//!Variable is in charge of command failed. This will be implemented in future firmware revisions. Variable is composed of one bit. 
		uint8 CF				: 1 ;	// bits  0		
		//!These bits are reserved. Variable is composed of three bits.
		uint8 RES				: 1 ;	// bit   1	
		//!Variable is in charge of batch length. Variable is composed of four bits. 
		uint8 BL				: 4 ;	// bits  2,3,4,5		
		//!Variable is in charge of batch operation. Variable is composed of one bit. 
		uint8 IsBatch			: 1 ;	// bit   6	
		//!Variable is in charge of data. Variable is composed of one bit. 
		uint8 HasData			: 1 ;	// bit   7	
	} sBitsPacketType;

	//!Union is responsible for byte structure of packet type.
	typedef union 
	{
		//!Variable is in charge of bits representation on packet type.
		sBitsPacketType			bits;
		//!Variable is in charge of byte representation on packet type.
		uint8					byte;
	} PacketType;
	//---------------- END: Packet Type ---------------------------------

	//------------------SERIAL PACKET STRUCTURE---------------------------
	//!Struct is responsible for SERIAL PACKET STRUCTURE.
	typedef struct 
	{
		//!Variable is in charge of header.
		uint8					header1; 
		//!Variable is in charge of header.
		uint8					header2;
		//!Variable is in charge of header.
		uint8					header3 ;
		//!Variable is in charge of packet type.
		PacketType				packet_type;
		//!Variable is in charge of address the register or command.
		uint8					address;
		//!Variable is in charge of data to send.
		uint8					data[16*4]; //zmienna do wys�ania
		//!Variable is in charge of checksum.
		uint8					cheksum1;
		//!Variable is in charge of checksum.
		uint8					cheksum0;
		//!Variable is in charge of data to write the registers.
		_array					data_bytes[16]; //zmienna do wpisania rejestru
	}Packet;
	//------------------END: SERIAL PACKET STRUCTURE---------------------------

	typedef struct {
		double x;
		double y;
		double z;
		eMsgFromUM6 status;
	}XYZ_Response;

	typedef struct {
		double phi;
		double psi;
		double theta;
		eMsgFromUM6 status;
	}Euler_Response;

	//!Class is responsible for all UM6 register.
	class Imu
	{
		public:
			
			//volatile bool fetch;
			//! Length of array called Register
			static const int length_array = 256;
			//! Serial port baud length
			static const int Baud = 115200;
			//--------------- CONFIGURATION REGISTER ---------------
			//! This is responsible for specifies communication settings on the UM6. This register stores a 32-bit integering point value.
			CommunicationRegister					CommRegister;
			//! This is responsible for miscellaneous configuration options. This register stores a 32-bit integering point value.
			MiscellaneousConfigurationRegister		MiscConfigRegister;
			
			//! Stores the values used to compensate for rate gyro bias on the X and Y gyro axes. This register stores a 32-bit integering point value.
			Gyro_Bias_XY_Register					G_Bias_XY_Register;
			//! Stores the values used to compensate for rate gyro bias on the rate gyro Z axis. This register stores a 32-bit integering point value.
			Gyro_Bias_Z_Register					G_Bias_Z_Register;
			//! Stores the values used to compensate for bias on the X and Y accelerometer axes. This register stores a 32-bit integering point value.
			Accel_Bias_XY_Register					A_Bias_XY_Register;
			//! Stores the values used to compensate for bias on the accelerometer Z axis. This register stores a 32-bit integering point value.
			Accel_Bias_Z_Register					A_Bias_Z_Register;
			//! Stores the values used to compensate for bias on the X and Y magnetometer axes. This register stores a 32-bit integering point value.
			Mag_Bias_XY_Register					M_Bias_XY_Register;
			//! Stores the values used to compensate for bias on the magnetometer Z axis. This register stores a 32-bit integering point value.
			Mag_Bias_Z_Register						M_Bias_Z_Register;

			//! X-component of the magnetic field reference vector.  This register stores a 32-bit floating point value. 
			_float2bytes							Mag_Ref_X;
			//! Y-component of the magnetic field reference vector.  This register stores a 32-bit floating point value. 
			_float2bytes							Mag_Ref_Y;
			//! Z-component of the magnetic field reference vector.  This register stores a 32-bit floating point value. 
			_float2bytes							Mag_Ref_Z;

			//!X-component of the accelerometer reference vector.  This register stores a 32-bit floating point value. 
			_float2bytes 							Accel_Ref_X;
			//!Y-component of the accelerometer reference vector.  This register stores a 32-bit floating point value. 
			_float2bytes 							Accel_Ref_Y;
			//!Z-component of the accelerometer reference vector.  This register stores a 32-bit floating point value. 
			_float2bytes 							Accel_Ref_Z;

			//! Variance of magnetometer noise. This register stores a 32-bit floating point value. 
			/** This value is used by the EKF during the magnetometer update 
			*step to compute the Kalman Gain and to propagate the error covariance.
			*/
			_float2bytes 							Ekf_Mag_Variance;

			//! Variance of accelerometer noise. This register stores a 32-bit floating point value.
			/*This value is used by the EKF during the accelerometer update 
			*step to compute the Kalman Gain and to propagate the error covariance.
			*/
			_float2bytes 							Ekf_Accel_Variance;

			//! Variance of process noise. This register stores a 32-bit floating point value.
			/*This value is used by the EKF during the predictions to propagate the error covariance. 
			*/
			_float2bytes							Ekf_Process_Variance;

			//!A set of 1 register storing the accelerometer calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Accel_Cal_00; 
			//!A set of 2 register storing the accelerometer calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Accel_Cal_01;
			//!A set of 3 register storing the accelerometer calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Accel_Cal_02; 
			//!A set of 4 register storing the accelerometer calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes							Accel_Cal_10;
			//!A set of 5 register storing the accelerometer calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Accel_Cal_11; 
			//!A set of 6 register storing the accelerometer calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Accel_Cal_12; 
			//!A set of 7 register storing the accelerometer calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Accel_Cal_20; 
			//!A set of 8 register storing the accelerometer calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Accel_Cal_21; 
			//!A set of 9 register storing the accelerometer calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Accel_Cal_22; 

			//!A set of 1 register storing the rate gyro calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Gyro_Cal_00; 
			//!A set of 2 register storing the rate gyro calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Gyro_Cal_01; 
			//!A set of 3 register storing the rate gyro calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Gyro_Cal_02; 
			//!A set of 4 register storing the rate gyro calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Gyro_Cal_10; 
			//!A set of 5 register storing the rate gyro calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes							Gyro_Cal_11; 
			//!A set of 6 register storing the rate gyro calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Gyro_Cal_12; 
			//!A set of 7 register storing the rate gyro calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Gyro_Cal_20; 
			//!A set of 8 register storing the rate gyro calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Gyro_Cal_21; 
			//!A set of 9 register storing the rate gyro calibration matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Gyro_Cal_22; 

			//!A set of 1 register storing the magnetometer calibration matrix. This register stores a 32-bit floating point value. 
			_float2bytes 							Mag_Cal_00; 
			//!A set of 2 register storing the magnetometer calibration matrix. This register stores a 32-bit floating point value. 
			_float2bytes 							Mag_Cal_01; 
			//!A set of 3 register storing the magnetometer calibration matrix. This register stores a 32-bit floating point value. 
			_float2bytes 							Mag_Cal_02; 
			//!A set of 4 register storing the magnetometer calibration matrix. This register stores a 32-bit floating point value. 
			_float2bytes 							Mag_Cal_10; 
			//!A set of 5 register storing the magnetometer calibration matrix. This register stores a 32-bit floating point value. 
			_float2bytes 							Mag_Cal_11; 
			//!A set of 6 register storing the magnetometer calibration matrix. This register stores a 32-bit floating point value. 
			_float2bytes 							Mag_Cal_12; 
			//!A set of 7 register storing the magnetometer calibration matrix. This register stores a 32-bit floating point value. 
			_float2bytes 							Mag_Cal_20; 
			//!A set of 8 register storing the magnetometer calibration matrix. This register stores a 32-bit floating point value. 
			_float2bytes 							Mag_Cal_21; 
			//!A set of 9 register storing the magnetometer calibration matrix. This register stores a 32-bit floating point value. 
			_float2bytes 							Mag_Cal_22; 
			//--------------- END: CONFIGURATION REGISTER ---------------

			//---------------- DATA REGISTER ---------------------------------
			//!  Stores results of self-test operations, and indicates when internal errors have occurred. This register stores a 32-bit integering point value.
			StatusRegister							StatRegister;
			//! Stores the most recent data acquired from the X and Y rate gyro axes. This register stores a 32-bit integering point value.
			Gyro_Raw_XY_Register					G_Raw_XY_Register;
			//! Stores the most recent data acquired from the Z rate gyro axis. This register stores a 32-bit integering point value.
			Gyro_Raw_Z_Register						G_Raw_Z_Register;
			//! Stores the most recent data acquired from the X and Y accelerometer axes. This register stores a 32-bit integering point value.
			Accel_Raw_XY_Register					A_Raw_XY_Register;
			//! Stores the most recent data acquired from the Z accelerometer axis. This register stores a 32-bit integering point value.
			Accel_Raw_Z_Register					A_Raw_Z_Register;
			//! Stores the most recent data acquired from the X and Y magnetometer axes. This register stores a 32-bit integering point value.
			Mag_Raw_XY_Register						M_Raw_XY_Register;
			//! Stores the most recent data acquired from the Z magnetometer axes. This register stores a 32-bit integering point value.
			Mag_Raw_Z_Register						M_Raw_Z_Register;
			//!Stores the most recent processed data acquired from the X and Y axis rate gyros. This register stores a 32-bit integering point value.
			Gyro_Proc_XY_Register					G_Proc_XY_Register;
			//! Stores the most recent processed data acquired from the Z axis rate gyro. This register stores a 32-bit integering point value.
			Gyro_Proc_Z_Register					G_Proc_Z_Register;
			//! Stores the most recent processed data acquired from the X and Y axis accelerometers. This register stores a 32-bit integering point value.
			Accel_Proc_XY_Register					A_Proc_XY_Register;
			//! Stores the most recent processed data acquired from the Z axis accelerometer. This register stores a 32-bit integering point value.
			Accel_Proc_Z_Register					A_Proc_Z_Register;
			//! Stores the most recent processed data acquired from the X and Y axis magnetometers. This register stores a 32-bit integering point value.
			Mag_Proc_XY_Register					M_Proc_XY_Register;
			//! Stores the most recent processed data acquired from the Z axis magnetometers. This register stores a 32-bit integering point value.
			Mag_Proc_Z_Register						M_Proc_Z_Register;
			//! Stores the most recently computed roll (phi) and pitch (theta) angle estimates. This register stores a 32-bit integering point value.
			Euler_Phi_Theta_Register				E_Phi_Theta_Register;
			//! Stores the most recently computed yaw (psi) angle estimate. This register stores a 32-bit integering point value.
			Euler_Psi_Register						E_Psi_Register;
			//! Stores the first two components of the most recent quaternion attitude estimate. This register stores a 32-bit integering point value.
			Quat_AB_Register						Q_AB_Register;
			//! Stores the second two components of the most recent quaternion attitude estimate. This register stores a 32-bit integering point value.
			Quat_CD_Register						Q_CD_Register;
			
			//!A set of 1 registers storing the error covariance matrix. This register stores a 32-bit floating point value. 
			_float2bytes 							Error_Cov_00;
			//!A set of 2 registers storing the error covariance matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Error_Cov_01;
			//!A set of 3 registers storing the error covariance matrix. This register stores a 32-bit floating point value.
			_float2bytes							Error_Cov_02;
			//!A set of 4 registers storing the error covariance matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Error_Cov_03;
			//!A set of 5 registers storing the error covariance matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Error_Cov_10;
			//!A set of 6 registers storing the error covariance matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Error_Cov_11;
			//!A set of 7 registers storing the error covariance matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Error_Cov_12;
			//!A set of 8 registers storing the error covariance matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Error_Cov_13;
			//!A set of 9 registers storing the error covariance matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Error_Cov_20;
			//!A set of 10 registers storing the error covariance matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Error_Cov_21;
			//!A set of 11 registers storing the error covariance matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Error_Cov_22;
			//!A set of 12 registers storing the error covariance matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Error_Cov_23;
			//!A set of 13 registers storing the error covariance matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Error_Cov_30;
			//!A set of 14 registers storing the error covariance matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Error_Cov_31;
			//!A set of 15 registers storing the error covariance matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Error_Cov_32;
			//!A set of 16 registers storing the error covariance matrix. This register stores a 32-bit floating point value.
			_float2bytes 							Error_Cov_33;
			//---------------- END: DATA REGISTER ---------------------------------

			//! Causes the UM6 to report the firmware revision. This register stores a 32-bit integering point value.
			GetFwVersionRegister					G_FwVersionRegister;
			
			//Array where the data is writing. 
			_array Register[length_array]; 

			//Write data to UM6.
			eMsgFromUM6 Write_To_UM6(Address A, uint8 BL = 1);
			//Read data from UM6.
			eMsgFromUM6 Read_From_UM6(Address A, uint8 BL = 0);
			
			//Write new configuration to UM6.
			eMsgFromUM6 WriteConfiguration();
			//Read configuration from UM6.
			eMsgFromUM6 ReadConfiguration();
			
			//Write new miscellaneous configuration to UM6.
			eMsgFromUM6 WriteMiscellaneous();
			//Read miscellaneous configuration from UM6.
			eMsgFromUM6 ReadMiscellaneous();
				
			//Read data from gyro.
			XYZ_Response GetGyro();
			//Read data from accelerometer.
			XYZ_Response GetAccel();
			//Read data from magnetometer.
			XYZ_Response GetMag();
			//Read Euler angles.
			Euler_Response GetEuler();

			//Enable broadcasting mode
			void EnableBroadcasting(bool accel, bool gyro, bool mag, bool euler);
			//Disable broadcasting mode
			void DisableBroadcasting();

			int8 testConnection();

			//Change serial port configuration.
			void ChangeSerialPortConfiguartion(eBaud baud, const std::string& device);

		public:
			Imu(unsigned int baud = Baud, const std::string& device = "COM1");
			~Imu(void);

		private:	
			bool				broadcasting_;		///< variable that indicates whether IMU is in broadcasting mode or not
			
			std::string			device_;	
			void				searchForDevice();	///< searches for a device and connects to it
			bool				connectionTestOk(); ///< tests if the connection to the controller was properly established
		
			boost::thread *		thread_;
			volatile bool		threadEnd;

			//!Variable is in charge of answer from UM6.
			eMsgFromUM6			raport;
			uint8				reportAddress;
			//Packet SerialPacketReadIMU;
			SerialPort*			serial_port; 

			//! Make array with all register.
			void ConfigArray();
			//! Add register to SERIAL PAKIET STRUCTURE.
			void addRegisterToSend(Packet & packet, _array rejestr[], Address A, uint8 BL);
			//!  Write data to array "data".
			void addDataToRegister(Packet & packet, uint8 A, uint8 BL);
			//! Write new data to Register.
			void UpdateRegister(uint8 rejestr[]);
			//! Send packet to UM6.
			void sendPacket(Packet packet, uint8 BL = 0);
			//! Get answer from UM6.
			void Get_Data_From_UM6();
	};
}

#endif //IMU_H

