#include "../include/Imu.h"
#include "../include/Address.h"
#include "../include/SerialPort.h"
#include <ctime>
#include <unistd.h>
#include <iostream>

using namespace std;
using namespace boost;

namespace trobot
{
	Imu::Imu(unsigned int baud, const string& device)
	{
		device_ = device;
		try {
			serial_port = new SerialPort(baud, device);
		}
		catch (std::exception e) {
			searchForDevice();
		}
		if(!connectionTestOk())
			searchForDevice();

		G_FwVersionRegister.FwVersion = 0;
		thread_ = new thread(bind(&Imu::Get_Data_From_UM6, this));
		//printf("Testing port");
		ConfigArray();

		broadcasting_ = false;
		//SerialPacketReadIMU.header1 = 's';
		//SerialPacketReadIMU.header2 = 'n';
		//SerialPacketReadIMU.header3 = 'p';
	}

	Imu::~Imu(void)
	{
		thread_->join();
		if(serial_port->isActive()) serial_port->close();
		delete serial_port;
	}

	void Imu::ChangeSerialPortConfiguartion(eBaud baud, const string& device)
	{
		CommRegister.Bytes.Byte1.bitsB1.baud_rate = baud;
		WriteConfiguration();
		thread_->join();
		delete [] thread_;
		if(!serial_port->isActive()) serial_port->close();
		delete serial_port;
		serial_port = new SerialPort(baud, device);
		thread_ = new thread(boost::bind(&Imu::Get_Data_From_UM6, this));
	}

	void Imu::searchForDevice()
	{
		std::vector<std::string> comList;
		SerialPort::DetectComPorts(comList, 128);

		for(vector<string>::iterator port = comList.begin(); port != comList.end(); port++)
		{
			try
			{
				serial_port = new SerialPort(Baud, *port);
			}
			catch(...)
			{
				continue;
			}
			device_ = *port;
			if( connectionTestOk() )
				return;
		}
	}

	void Imu::ConfigArray()
	{
		Register[COMMUNICATION]._int			=	CommRegister.Communication;
		Register[MISC_CONFIG]._int				=	MiscConfigRegister.Miscellaneous;

		Register[MAG_REF_X]._float				=	Mag_Ref_X;
		Register[MAG_REF_Y]._float				=	Mag_Ref_Y;
		Register[MAG_REF_Z]._float				=	Mag_Ref_Z;

		Register[ACCEL_REF_X]._float			=	Accel_Ref_X;
		Register[ACCEL_REF_Y]._float			=	Accel_Ref_Y;
		Register[ACCEL_REF_Z]._float			=	Accel_Ref_Z;

		Register[EKF_MAG_VARIANCE]._float		=	Ekf_Mag_Variance;
		Register[EKF_ACCEL_VARIANCE]._float		=	Ekf_Accel_Variance;
		Register[EKF_PROCESS_VARIANCE]._float	=	Ekf_Process_Variance;

		Register[GYRO_BIAS_XY]._int				=	G_Bias_XY_Register.Gyro_Bias_XY;
		Register[GYRO_BIAS_Z]._int				=	G_Bias_Z_Register.Gyro_Bias_Z;
		Register[ACCEL_BIAS_XY]._int			=	A_Bias_XY_Register.Accel_Bias_XY;
		Register[ACCEL_BIAS_Z]._int				=	A_Bias_Z_Register.Accel_Bias_Z;
		Register[MAG_BIAS_XY]._int				=	M_Bias_XY_Register.Mag_Bias_XY;
		Register[MAG_BIAS_Z]._int				=	M_Bias_Z_Register.Mag_Bias_Z;

		Register[ACCEL_CAL_00]._float			=	Accel_Cal_00;
		Register[ACCEL_CAL_01]._float			=	Accel_Cal_01;
		Register[ACCEL_CAL_02]._float			=	Accel_Cal_02;
		Register[ACCEL_CAL_10]._float			=	Accel_Cal_10;
		Register[ACCEL_CAL_11]._float			=	Accel_Cal_11;
		Register[ACCEL_CAL_12]._float			=	Accel_Cal_12;
		Register[ACCEL_CAL_20]._float			=	Accel_Cal_20;
		Register[ACCEL_CAL_21]._float			=	Accel_Cal_21;
		Register[ACCEL_CAL_22]._float			=	Accel_Cal_22;

		Register[GYRO_CAL_00]._float			=	Gyro_Cal_00;
		Register[GYRO_CAL_01]._float			=	Gyro_Cal_01;
		Register[GYRO_CAL_02]._float			=	Gyro_Cal_02;
		Register[GYRO_CAL_10]._float			=	Gyro_Cal_10;
		Register[GYRO_CAL_11]._float			=	Gyro_Cal_11;
		Register[GYRO_CAL_12]._float			=	Gyro_Cal_12;
		Register[GYRO_CAL_20]._float			=	Gyro_Cal_20;
		Register[GYRO_CAL_21]._float			=	Gyro_Cal_21;
		Register[GYRO_CAL_22]._float			=	Gyro_Cal_22;

		Register[MAG_CAL_00]._float				=	Mag_Cal_00;
		Register[MAG_CAL_01]._float				=	Mag_Cal_01;
		Register[MAG_CAL_02]._float				=	Mag_Cal_02;
		Register[MAG_CAL_10]._float				=	Mag_Cal_10;
		Register[MAG_CAL_11]._float				=	Mag_Cal_11;
		Register[MAG_CAL_12]._float				=	Mag_Cal_12;
		Register[MAG_CAL_20]._float				=	Mag_Cal_20;
		Register[MAG_CAL_21]._float				=	Mag_Cal_21;
		Register[MAG_CAL_22]._float				=	Mag_Cal_22;

		Register[STATUS]._int					=	StatRegister.Status;

		Register[GYRO_RAW_XY]._int				=	G_Raw_XY_Register.Gyro_Raw_XY;
		Register[GYRO_RAW_Z]._int				=	G_Raw_Z_Register.Gyro_Raw_Z;
		Register[ACCEL_RAW_XY]._int				=	A_Raw_XY_Register.Accel_Raw_XY;
		Register[ACCEL_RAW_Z]._int				=	A_Raw_Z_Register.Accel_Raw_Z;
		Register[MAG_RAW_XY]._int				=	M_Raw_XY_Register.Mag_Raw_XY;
		Register[MAG_RAW_Z]._int				=	M_Raw_Z_Register.Mag_Raw_Z;

		Register[GYRO_PROC_XY]._int				=	G_Proc_XY_Register.Gyro_Proc_XY;
		Register[GYRO_PROC_Z]._int				=	G_Proc_Z_Register.Gyro_Proc_Z;
		Register[ACCEL_PROC_XY]._int			=	A_Proc_XY_Register.Accel_Proc_XY;
		Register[ACCEL_PROC_Z]._int				=	A_Proc_Z_Register.Accel_Proc_Z;
		Register[MAG_PROC_XY]._int				=	M_Proc_XY_Register.Mag_Proc_XY;
		Register[MAG_PROC_Z]._int				=	M_Proc_Z_Register.Mag_Proc_Z;

		Register[EULER_PHI_THETA]._int			=	E_Phi_Theta_Register.Euler_Phi_Theta;
		Register[EULER_PSI]._int				=	E_Psi_Register.Euler_Psi;
		Register[QUAT_AB]._int					=	Q_AB_Register.Quat_AB;
		Register[QUAT_CD]._int					=	Q_CD_Register.Quat_CD;

		Register[ERROR_COV_00]._float			=	Error_Cov_00;
		Register[ERROR_COV_01]._float			=	Error_Cov_01;
		Register[ERROR_COV_02]._float			=	Error_Cov_02;
		Register[ERROR_COV_03]._float			=	Error_Cov_03;
		Register[ERROR_COV_10]._float			=	Error_Cov_10;
		Register[ERROR_COV_11]._float			=	Error_Cov_11;
		Register[ERROR_COV_12]._float			=	Error_Cov_12;
		Register[ERROR_COV_13]._float			=	Error_Cov_13;
		Register[ERROR_COV_20]._float			=	Error_Cov_20;
		Register[ERROR_COV_21]._float			=	Error_Cov_21;
		Register[ERROR_COV_22]._float			=	Error_Cov_22;
		Register[ERROR_COV_23]._float			=	Error_Cov_23;
		Register[ERROR_COV_30]._float			=	Error_Cov_30;
		Register[ERROR_COV_31]._float			=	Error_Cov_31;
		Register[ERROR_COV_32]._float			=	Error_Cov_32;
		Register[ERROR_COV_33]._float			=	Error_Cov_33;

		Register[GET_FW_VERSION]._int			=	G_FwVersionRegister.FwVersion;
		Register[FLASH_COMMIT]._int				=	0;
		Register[ZERO_GYROS]._int				=	0;
		Register[RESET_EKF]._int				=	0;
		Register[GET_DATA]._int					=	0;
		Register[SET_ACCEL_REF]._int			=	0;
		Register[SET_MAG_REF]._int				=	0;
		Register[RESET_TO_FACTORY]._int			=	0;
		Register[BAD_CHECKSUM]._int				=	0;
		Register[UNKNOWN_ADDRESS]._int			=	0;
		Register[INVALID_BATCH_SIZE]._int		=	0;
	}

	void Imu::UpdateRegister(uint8 rejestr[])
	{
		for(int i=0; i<16; i++)
		{
			if(rejestr[i] == COMMUNICATION)
				CommRegister.Communication			=	Register[COMMUNICATION]._int;
			if(rejestr[i] == MISC_CONFIG)
				MiscConfigRegister.Miscellaneous	=	Register[MISC_CONFIG]._int;
			if(rejestr[i] == GYRO_BIAS_XY)
				G_Bias_XY_Register.Gyro_Bias_XY		=	Register[GYRO_BIAS_XY]._int;
			if(rejestr[i] == GYRO_BIAS_Z)
				G_Bias_Z_Register.Gyro_Bias_Z		=	Register[GYRO_BIAS_Z]._int;
			if(rejestr[i] == ACCEL_BIAS_XY)
				A_Bias_XY_Register.Accel_Bias_XY	=	Register[ACCEL_BIAS_XY]._int;
			if(rejestr[i] == ACCEL_BIAS_Z)
				A_Bias_Z_Register.Accel_Bias_Z		=	Register[ACCEL_BIAS_Z]._int;
			if(rejestr[i] == MAG_BIAS_XY)
				M_Bias_XY_Register.Mag_Bias_XY		=	Register[MAG_BIAS_XY]._int;
			if(rejestr[i] == MAG_BIAS_Z)
				M_Bias_Z_Register.Mag_Bias_Z		=	Register[MAG_BIAS_Z]._int;

			if(rejestr[i] == STATUS)
				StatRegister.Status					=	Register[STATUS]._int;
			if(rejestr[i] == GYRO_RAW_XY)
				G_Raw_XY_Register.Gyro_Raw_XY		=	Register[GYRO_RAW_XY]._int;
			if(rejestr[i] == GYRO_RAW_Z)
				G_Raw_Z_Register.Gyro_Raw_Z			=	Register[GYRO_RAW_Z]._int;
			if(rejestr[i] == ACCEL_RAW_XY)
				A_Raw_XY_Register.Accel_Raw_XY		=	Register[ACCEL_RAW_XY]._int;
			if(rejestr[i] == ACCEL_RAW_Z)
				A_Raw_Z_Register.Accel_Raw_Z		=	Register[ACCEL_RAW_Z]._int;
			if(rejestr[i] == MAG_RAW_XY)
				M_Raw_XY_Register.Mag_Raw_XY		=	Register[MAG_RAW_XY]._int;
			if(rejestr[i] == MAG_RAW_Z)
				M_Raw_Z_Register.Mag_Raw_Z			=	Register[MAG_RAW_Z]._int;

			if(rejestr[i] == GYRO_PROC_XY)
				G_Proc_XY_Register.Gyro_Proc_XY		=	Register[GYRO_PROC_XY]._int;
			if(rejestr[i] == GYRO_PROC_Z)
				G_Proc_Z_Register.Gyro_Proc_Z		=	Register[GYRO_PROC_Z]._int;
			if(rejestr[i] == ACCEL_PROC_XY)
				A_Proc_XY_Register.Accel_Proc_XY	=	Register[ACCEL_PROC_XY]._int;
			if(rejestr[i] == ACCEL_PROC_Z)
				A_Proc_Z_Register.Accel_Proc_Z		=	Register[ACCEL_PROC_Z]._int;
			if(rejestr[i] == MAG_PROC_XY)
				M_Proc_XY_Register.Mag_Proc_XY		=	Register[MAG_PROC_XY]._int;
			if(rejestr[i] == MAG_PROC_Z)
				M_Proc_Z_Register.Mag_Proc_Z		=	Register[MAG_PROC_Z]._int;

			if(rejestr[i] == EULER_PHI_THETA)
				E_Phi_Theta_Register.Euler_Phi_Theta=	Register[EULER_PHI_THETA]._int;
			if(rejestr[i] == EULER_PSI)
				E_Psi_Register.Euler_Psi			=	Register[EULER_PSI]._int;
			if(rejestr[i] == QUAT_AB)
				Q_AB_Register.Quat_AB				=	Register[QUAT_AB]._int;
			if(rejestr[i] == QUAT_CD)
				Q_CD_Register.Quat_CD				=	Register[QUAT_CD]._int;

			if(rejestr[i] == MAG_REF_X)
				Mag_Ref_X							=	Register[MAG_REF_X]._float;
			if(rejestr[i] == MAG_REF_Y)
				Mag_Ref_Y							=	Register[MAG_REF_Y]._float;
			if(rejestr[i] == MAG_REF_Z)
				Mag_Ref_Z							=	Register[MAG_REF_Z]._float;

			if(rejestr[i] == ACCEL_REF_X)
				Accel_Ref_X							=	Register[ACCEL_REF_X]._float;
			if(rejestr[i] == ACCEL_REF_Y)
				Accel_Ref_Y							=	Register[ACCEL_REF_Y]._float;
			if(rejestr[i] == ACCEL_REF_Z)
				Accel_Ref_Z							=	Register[ACCEL_REF_Z]._float;

			if(rejestr[i] == EKF_MAG_VARIANCE)
				Ekf_Mag_Variance					=	Register[EKF_MAG_VARIANCE]._float;
			if(rejestr[i] == EKF_ACCEL_VARIANCE)
				Ekf_Accel_Variance					=	Register[EKF_ACCEL_VARIANCE]._float;
			if(rejestr[i] == EKF_PROCESS_VARIANCE)
				Ekf_Process_Variance				=	Register[EKF_PROCESS_VARIANCE]._float;

			if(rejestr[i] == ACCEL_CAL_00)
				Accel_Cal_00						=	Register[ACCEL_CAL_00]._float;
			if(rejestr[i] == ACCEL_CAL_01)
				Accel_Cal_01						=	Register[ACCEL_CAL_01]._float;
			if(rejestr[i] == ACCEL_CAL_02)
				Accel_Cal_02						=	Register[ACCEL_CAL_02]._float;
			if(rejestr[i] == ACCEL_CAL_10)
				Accel_Cal_10						=	Register[ACCEL_CAL_10]._float;
			if(rejestr[i] == ACCEL_CAL_11)
				Accel_Cal_11						=	Register[ACCEL_CAL_11]._float;
			if(rejestr[i] == ACCEL_CAL_12)
				Accel_Cal_12						=	Register[ACCEL_CAL_12]._float;
			if(rejestr[i] == ACCEL_CAL_20)
				Accel_Cal_20						=	Register[ACCEL_CAL_20]._float;
			if(rejestr[i] = ACCEL_CAL_21)
				Accel_Cal_21						=	Register[ACCEL_CAL_21]._float;
			if(rejestr[i] == ACCEL_CAL_22)
				Accel_Cal_22						=	Register[ACCEL_CAL_22]._float;

			if(rejestr[i] == GYRO_CAL_00)
				Gyro_Cal_00							=	Register[GYRO_CAL_00]._float;
			if(rejestr[i] == GYRO_CAL_01)
				Gyro_Cal_01							=	Register[GYRO_CAL_01]._float;
			if(rejestr[i] == GYRO_CAL_02)
				Gyro_Cal_02							=	Register[GYRO_CAL_02]._float;
			if(rejestr[i] == GYRO_CAL_10)
				Gyro_Cal_10							=	Register[GYRO_CAL_10]._float;
			if(rejestr[i] == GYRO_CAL_11)
				Gyro_Cal_11							=	Register[GYRO_CAL_11]._float;
			if(rejestr[i] == GYRO_CAL_12)
				Gyro_Cal_12							=	Register[GYRO_CAL_12]._float;
			if(rejestr[i] == GYRO_CAL_20)
				Gyro_Cal_20							=	Register[GYRO_CAL_20]._float;
			if(rejestr[i] == GYRO_CAL_21)
				Gyro_Cal_21							=	Register[GYRO_CAL_21]._float;
			if(rejestr[i] == GYRO_CAL_22)
				Gyro_Cal_22							=	Register[GYRO_CAL_22]._float;

			if(rejestr[i] == MAG_CAL_00)
				Mag_Cal_00							=	Register[MAG_CAL_00]._float;
			if(rejestr[i] == MAG_CAL_01)
				Mag_Cal_01							=	Register[MAG_CAL_01]._float;
			if(rejestr[i] == MAG_CAL_02)
				Mag_Cal_02							=	Register[MAG_CAL_02]._float;
			if(rejestr[i] == MAG_CAL_10)
				Mag_Cal_10							=	Register[MAG_CAL_10]._float;
			if(rejestr[i] == MAG_CAL_11)
				Mag_Cal_11							=	Register[MAG_CAL_11]._float;
			if(rejestr[i] == MAG_CAL_12)
				Mag_Cal_12							=	Register[MAG_CAL_12]._float;
			if(rejestr[i] == MAG_CAL_20)
				Mag_Cal_20							=	Register[MAG_CAL_20]._float;
			if(rejestr[i] == MAG_CAL_21)
				Mag_Cal_21							=	Register[MAG_CAL_21]._float;
			if(rejestr[i] == MAG_CAL_22)
				Mag_Cal_22							=	Register[MAG_CAL_22]._float;

			if(rejestr[i] == ERROR_COV_00)
				Error_Cov_00						=	Register[ERROR_COV_00]._float;
			if(rejestr[i] == ERROR_COV_01)
				Error_Cov_01						=	Register[ERROR_COV_01]._float;
			if(rejestr[i] == ERROR_COV_02)
				Error_Cov_02						=	Register[ERROR_COV_02]._float;
			if(rejestr[i] == ERROR_COV_03)
				Error_Cov_03						=   Register[ERROR_COV_03]._float;
			if(rejestr[i] == ERROR_COV_10)
				Error_Cov_10						=	Register[ERROR_COV_10]._float;
			if(rejestr[i] == ERROR_COV_11)
				Error_Cov_11						=	Register[ERROR_COV_11]._float;
			if(rejestr[i] == ERROR_COV_12)
				Error_Cov_12						=	Register[ERROR_COV_12]._float;
			if(rejestr[i] == ERROR_COV_13)
				Error_Cov_13						=	Register[ERROR_COV_13]._float;
			if(rejestr[i] == ERROR_COV_20)
				Error_Cov_20						=	Register[ERROR_COV_20]._float;
			if(rejestr[i] == ERROR_COV_21)
				Error_Cov_21						=	Register[ERROR_COV_21]._float;
			if(rejestr[i] == ERROR_COV_22)
				Error_Cov_22						=	Register[ERROR_COV_22]._float;
			if(rejestr[i] == ERROR_COV_23)
				Error_Cov_23						=	Register[ERROR_COV_23]._float;
			if(rejestr[i] == ERROR_COV_30)
				Error_Cov_30						=	Register[ERROR_COV_30]._float;
			if(rejestr[i] == ERROR_COV_31)
				Error_Cov_31						=	Register[ERROR_COV_31]._float;
			if(rejestr[i] == ERROR_COV_32)
				Error_Cov_32						=	Register[ERROR_COV_32]._float;
			if(rejestr[i] == ERROR_COV_33)
				Error_Cov_33						=	Register[ERROR_COV_33]._float;

			if(rejestr[i] == GET_FW_VERSION)
				G_FwVersionRegister.FwVersion		=	Register[GET_FW_VERSION]._int;
		}
	}

	eMsgFromUM6 Imu::Write_To_UM6(Address A, uint8 BL)
	{
		ConfigArray();

		Packet SerialPacketWriteIMU;

		SerialPacketWriteIMU.header1 = 's';
		SerialPacketWriteIMU.header2 = 'n';
		SerialPacketWriteIMU.header3 = 'p';

		SerialPacketWriteIMU.address = A;

		SerialPacketWriteIMU.packet_type.byte = 0;
		SerialPacketWriteIMU.packet_type.bits.HasData = 1;
		SerialPacketWriteIMU.packet_type.bits.BL = BL;

		//SerialPacketWriteIMU.cheksum1 = 0;
		//SerialPacketWriteIMU.cheksum0 = 0;

		if(SerialPacketWriteIMU.packet_type.bits.BL == 1)
		{
			SerialPacketWriteIMU.packet_type.bits.IsBatch = 0;
		}

		else if(SerialPacketWriteIMU.packet_type.bits.BL > 1)
		{
			SerialPacketWriteIMU.packet_type.bits.IsBatch = 1;
		}

		for(int j = 0; j < BL; j++)
		{
			SerialPacketWriteIMU.data_bytes[j] = Register[SerialPacketWriteIMU.address + j];
		}

		addRegisterToSend(SerialPacketWriteIMU, SerialPacketWriteIMU.data_bytes, A, BL);
		sendPacket(SerialPacketWriteIMU, BL);

		int duration = 0;
		while(raport == _NOTHING || (reportAddress != A && reportAddress < BAD_CHECKSUM))
		{
			if(duration >= 100)
			{
				return raport;
			}
			duration++;
			usleep(5000);
		}

		eMsgFromUM6 tmp = raport;
		raport = _NOTHING;

		return tmp;
	}

	eMsgFromUM6 Imu::Read_From_UM6(Address A, uint8 BL)
	{
		Packet SerialPacketWriteIMU;

		SerialPacketWriteIMU.header1 = 's';
		SerialPacketWriteIMU.header2 = 'n';
		SerialPacketWriteIMU.header3 = 'p';

		SerialPacketWriteIMU.address = A;

		SerialPacketWriteIMU.packet_type.byte = 0;
		SerialPacketWriteIMU.packet_type.bits.HasData = 0;
		SerialPacketWriteIMU.packet_type.bits.BL = BL;

		if(SerialPacketWriteIMU.packet_type.bits.BL == 1 && SerialPacketWriteIMU.packet_type.bits.BL == 0)
			SerialPacketWriteIMU.packet_type.bits.IsBatch = 0;

		else if(SerialPacketWriteIMU.packet_type.bits.BL > 1)
			SerialPacketWriteIMU.packet_type.bits.IsBatch = 1;

		SerialPacketWriteIMU.packet_type.byte = 0;

		//SerialPacketWriteIMU.cheksum1 = 0;
		//SerialPacketWriteIMU.cheksum0 = 0;

		sendPacket(SerialPacketWriteIMU);

		int duration = 0;

		while(raport == _NOTHING || (reportAddress != A && reportAddress < BAD_CHECKSUM))
		{
			if(duration >= 100)
			{
				return raport;
			}
			duration++;
			usleep(5000);
		}

		eMsgFromUM6 tmp = raport;
		raport = _NOTHING;

		return tmp;
	}

	void Imu::addRegisterToSend(Packet & packet, _array rejestr[], Address A, uint8 BL)
	{
		for(int j = 0; j < (BL*4); j += 4)
		{
			if( (A == COMMUNICATION)							    ||
				(A == MISC_CONFIG)									||
				((A >= GYRO_BIAS_XY) && (A <= MAG_BIAS_Z))			||
				((A >= STATUS) && (A <= QUAT_CD))					||
				((A >= GET_FW_VERSION) && (A <= RESET_TO_FACTORY))  ||
				((A >= BAD_CHECKSUM) && (A <= INVALID_BATCH_SIZE))
				)
			{
				uint_32 temporary;
				temporary.uint = rejestr[j]._int;
				packet.data[0 + j] = temporary.bytes.byte4;
				packet.data[1 + j] = temporary.bytes.byte3;
				packet.data[2 + j] = temporary.bytes.byte2;
				packet.data[3 + j] = temporary.bytes.byte1;
			}

			else
				if( ((A >= MAG_REF_X)	 && (A <= EKF_PROCESS_VARIANCE)) ||
					((A >= ACCEL_CAL_00) && (A <= MAG_CAL_22))			 ||
					((A >= ERROR_COV_00) && (A <= ERROR_COV_33))
			  )
				{
					packet.data[0 + j] = rejestr[j]._float.bytes.byte4;
					packet.data[1 + j] = rejestr[j]._float.bytes.byte3;
					packet.data[2 + j] = rejestr[j]._float.bytes.byte2;
					packet.data[3 + j] = rejestr[j]._float.bytes.byte1;
				}
		}
	}

	void Imu::addDataToRegister(Packet &packet, uint8 A, uint8 BL)
	{
		for(int j = 0; j < (BL*1); j ++)
		{
			if( (A == COMMUNICATION)							    ||
				(A == MISC_CONFIG)									||
				((A >= GYRO_BIAS_XY) && (A <= MAG_BIAS_Z))			||
				((A >= STATUS) && (A <= QUAT_CD))					||
				((A >= GET_FW_VERSION) && (A <= RESET_TO_FACTORY))  ||
				((A >= BAD_CHECKSUM) && (A <= INVALID_BATCH_SIZE))
				)
			{
				uint_32 temporary;
				temporary.bytes.byte4 = packet.data[0 + j*4];
				temporary.bytes.byte3 = packet.data[1 + j*4];
				temporary.bytes.byte2 = packet.data[2 + j*4];
				temporary.bytes.byte1 = packet.data[3 + j*4];
				Register[A + j]._int = temporary.uint;
			}

			else if( ((A >= MAG_REF_X)	 && (A <= EKF_PROCESS_VARIANCE)) ||
				((A >= ACCEL_CAL_00) && (A <= MAG_CAL_22))			 ||
				((A >= ERROR_COV_00) && (A <= ERROR_COV_33))
				)
			{
				Register[A + j]._float.bytes.byte4 = packet.data[0 + j*4];
				Register[A + j]._float.bytes.byte3 = packet.data[1 + j*4];
				Register[A + j]._float.bytes.byte2 = packet.data[2 + j*4];
				Register[A + j]._float.bytes.byte1 = packet.data[3 + j*4];
			}
		}
	}

	void Imu::sendPacket(Packet packet, uint8 BL)
	{
		uint16 checksum = 0;

		serial_port->write(packet.header1);
		checksum += packet.header1;
		serial_port->write(packet.header2);
		checksum += packet.header2;
		serial_port->write(packet.header3);
		checksum += packet.header3;
		serial_port->write(packet.packet_type.byte);
		checksum += packet.packet_type.byte;
		serial_port->write(packet.address);
		checksum += packet.address;
		//petla for wysylajaca dane
		for(int i = 0; i < (BL*4); i++)
		{
			serial_port->write(packet.data[i]);
			checksum += packet.data[i];
		}

		packet.cheksum1 = (checksum >> 8) & 0xff;
		packet.cheksum0 = checksum & 0xff;

		serial_port->write(packet.cheksum1);
		serial_port->write(packet.cheksum0);
	}

	eMsgFromUM6 Imu::WriteConfiguration()
	{
		return Write_To_UM6(COMMUNICATION, 1);
	}

	eMsgFromUM6 Imu::ReadConfiguration()
	{
		return Read_From_UM6(COMMUNICATION, 1);
	}

	eMsgFromUM6 Imu::WriteMiscellaneous()
	{
		return Write_To_UM6(MISC_CONFIG, 1);
	}

	eMsgFromUM6 Imu::ReadMiscellaneous()
	{
		return Read_From_UM6(MISC_CONFIG, 1);
	}

	XYZ_Response Imu::GetGyro()
	{
		XYZ_Response response;
		if(broadcasting_) {
			response.status = _READ_COMPLETE;			
		}
		else {
			response.status = Read_From_UM6(GYRO_PROC_XY, 4);
		}
		response.x = G_Proc_XY_Register.gyro_proc_xy.G_Proc_X * 0.0610352;
		response.y = G_Proc_XY_Register.gyro_proc_xy.G_Proc_Y * 0.0610352;
		response.z = G_Proc_Z_Register.gyro_p_z.G_Proc_Z * 0.0610352;
		return response;
	}

	XYZ_Response Imu::GetAccel()
	{
		XYZ_Response response;
		if(broadcasting_) {
			response.status = _READ_COMPLETE;			
		}
		else {
			response.status = Read_From_UM6(ACCEL_PROC_XY, 4);
		}
		response.x = A_Proc_XY_Register.accel_p_xy.A_Proc_X * 0.000183105;
		response.y = A_Proc_XY_Register.accel_p_xy.A_Proc_Y * 0.000183105;
		response.z = A_Proc_Z_Register.accel_p_z.A_Proc_Z * 0.000183105;
		return response;
		
	}

	XYZ_Response Imu::GetMag()
	{
		XYZ_Response response;
		if(broadcasting_) {
			response.status = _READ_COMPLETE;			
		}
		else {
			//response.status = Read_From_UM6(MAG_PROC_XY, 8);
		}
		response.x = M_Proc_XY_Register.mag_p_xy.M_Proc_X * 0.000305176;
		response.y = M_Proc_XY_Register.mag_p_xy.M_Proc_Y * 0.000305176;
		response.z = M_Proc_Z_Register.mag_p_z.M_Proc_Z * 0.000305176;
		return response;
		

	}
	Euler_Response Imu::GetEuler() 
	{
		Euler_Response response;
		if(broadcasting_) 
		{
			response.status = _READ_COMPLETE;
		}
		else
		{
			response.status = Read_From_UM6(EULER_PHI_THETA, 4);
		}
		response.phi = (double)E_Phi_Theta_Register.euler_p_t.Phi * 0.0109863;
		response.theta = (double)E_Phi_Theta_Register.euler_p_t.Theta * 0.0109863;
		response.psi = (double)E_Psi_Register.euler_p.Psi * 0.0109863;
		return response;
	}

	void Imu::Get_Data_From_UM6()
	{
		while(true) {
			
		if (serial_port->newDataAvailable()) 
		{
			
			const circular_buffer<char>  data = serial_port->getDataRead();
			int length = data.size();
			int packet_length = 1;
			uint8 address_array[16];
			Packet SerialPacketReadIMU;

			raport = _NOTHING;
			//if(serial_port->newDataAvailable())
			for(int i = 0; i < length - packet_length; i += packet_length)
			{
				if(	   (data[i]		== 's')
					&& (data[i + 1] == 'n')
					&& (data[i + 2] == 'p'))
				{
					SerialPacketReadIMU.packet_type.byte = data[i+3];
					SerialPacketReadIMU.address = data[i+4];

					if(SerialPacketReadIMU.packet_type.byte != 0 && SerialPacketReadIMU.packet_type.bits.HasData == 1)
					{

						reportAddress = SerialPacketReadIMU.address;

						if(SerialPacketReadIMU.packet_type.bits.IsBatch == 0)
						{
							address_array[0] = SerialPacketReadIMU.address;

							for(int h = 0; h <4; h++)
								SerialPacketReadIMU.data[h] = data[i+5+h];

							packet_length = 11;
						}

						else if(SerialPacketReadIMU.packet_type.bits.IsBatch == 1)
						{
							for(int j = 0; j <SerialPacketReadIMU.packet_type.bits.BL; j++)
							{
								address_array[j] = SerialPacketReadIMU.address + j;

								for(int h = 0; h <4; h++)
									SerialPacketReadIMU.data[j*4+h] = data[h+j*4+i+5];
							}

							packet_length = 7 + 4 * SerialPacketReadIMU.packet_type.bits.BL ;
						}

						addDataToRegister(SerialPacketReadIMU, SerialPacketReadIMU.address, SerialPacketReadIMU.packet_type.bits.BL);
						UpdateRegister(address_array);
						raport = _READ_COMPLETE;
					}

					else if(SerialPacketReadIMU.packet_type.byte == 0)
					{
						packet_length = 7;
						raport = _WRITE_COMPLETE;
					}

					else if(SerialPacketReadIMU.address == BAD_CHECKSUM)
					{
						packet_length = 7;
						raport = _BAD_CHEKSUM;
					}

					else if(SerialPacketReadIMU.address == UNKNOWN_ADDRESS)
					{
						packet_length = 7;
						raport = _UNKNOWN_ADDRESS;
					}

					else if(SerialPacketReadIMU.address == INVALID_BATCH_SIZE)
					{
						packet_length = 7;
						raport = _INVALID_BATCH_SIZE;
					}

					else if(SerialPacketReadIMU.packet_type.byte == 0)
					{
						packet_length = 7;
						raport = _COMMAND_COMPLETE;
					}

					else if(SerialPacketReadIMU.packet_type.bits.CF == 1)
					{
						packet_length = 7;
						raport = _COMMAND_FAILED;
					}
				}
			}
		}
		posix_time::milliseconds time(20);
		this_thread::sleep(time);
		}
	}

	int8 Imu::testConnection()
	{

		if(Read_From_UM6(GET_FW_VERSION) != _READ_COMPLETE)
		{
			return 0;
		}
		return 1;
	}

	bool Imu::connectionTestOk()
	{
		Write_To_UM6(GET_FW_VERSION,1);
		int i = 0;
		int timeoutCount = 10;
		do
		{
		usleep(100000);
		i++;
		}while( (i < timeoutCount) && (!serial_port->newDataAvailable()));

		circular_buffer<char> data = serial_port->getDataRead();


		int result = searchBufferR(data, "snp"); //looking for a identification string
		if (result >= 0) {
			cout << "IMU connected to: " << device_ << endl;
			return true;
		}
		else {
			serial_port->close();
			return false;
		}

		
	}

	void Imu::EnableBroadcasting(bool accel, bool gyro, bool mag, bool euler) 
	{
		CommRegister.Bytes.Byte0 = 27; // frequency of around 50Hz

		CommRegister.Bytes.Byte1.bitsB1.baud_rate = _BAUD_115200;

		CommRegister.Bytes.Byte2.bitsB2.COV = 0;
		CommRegister.Bytes.Byte2.bitsB2.EU = euler;
		CommRegister.Bytes.Byte2.bitsB2.QT = 0;

		CommRegister.Bytes.Byte3.bitsB3.AP = accel;
		CommRegister.Bytes.Byte3.bitsB3.AR = 0;
		CommRegister.Bytes.Byte3.bitsB3.BEN = 1;
		CommRegister.Bytes.Byte3.bitsB3.GP = gyro;
		CommRegister.Bytes.Byte3.bitsB3.GR = 0;
		CommRegister.Bytes.Byte3.bitsB3.MP = mag;
		CommRegister.Bytes.Byte3.bitsB3.MR = 0;

		Write_To_UM6(COMMUNICATION,1);

		broadcasting_ = true;
	}

	void Imu::DisableBroadcasting() 
	{
		CommRegister.Bytes.Byte1.bitsB1.baud_rate = _BAUD_115200;

		CommRegister.Bytes.Byte3.bitsB3.BEN = 0;

		Write_To_UM6(COMMUNICATION, 1);

		broadcasting_ = false;
	}



}
