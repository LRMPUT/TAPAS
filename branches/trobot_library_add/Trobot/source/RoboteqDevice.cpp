#include "RoboteqDevice.h"
#include "ErrorCodes.h"


using namespace std;

namespace trobot {


#define BUFFER_SIZE 1024
#define MISSING_VALUE -1024

	RoboteqDevice::RoboteqDevice(const string& device, unsigned int baud = 115200)
	{
		baud_ = baud;
		try {
			serialPort_ = new SerialPort(baud, device);
		}
		catch (std::exception e) {
			searchForDevice();
		}
		if(!connectionTestOk())
			searchForDevice();
		else
			cout << "Roboteq driver connected!\n";

	}
	RoboteqDevice::~RoboteqDevice()
	{

	}

	bool RoboteqDevice::IsConnected()
	{

		return serialPort_->isActive();
	}


	/*
	void RoboteqDevice::InitPort()
	{
	if(!IsConnected())
	return;

	DCB          comSettings;
	COMMTIMEOUTS CommTimeouts;

	// Set timeouts in milliseconds
	CommTimeouts.ReadIntervalTimeout         = 0;
	CommTimeouts.ReadTotalTimeoutMultiplier  = 0;
	CommTimeouts.ReadTotalTimeoutConstant    = 100;
	CommTimeouts.WriteTotalTimeoutMultiplier = 0;
	CommTimeouts.WriteTotalTimeoutConstant   = 100;
	SetCommTimeouts((HANDLE) handle,&CommTimeouts);

	// Set Port parameters.
	// Make a call to GetCommState() first in order to fill
	// the comSettings structure with all the necessary values.
	// Then change the ones you want and call SetCommState().
	GetCommState((HANDLE) handle, &comSettings);
	comSettings.BaudRate = 115200;
	comSettings.StopBits = ONESTOPBIT;
	comSettings.ByteSize = 8;
	comSettings.Parity   = NOPARITY;
	comSettings.fParity  = FALSE;
	SetCommState((HANDLE) handle, &comSettings);
	}*/

	int RoboteqDevice::Write(string str)
	{
		if(!IsConnected())
			return RQ_ERR_NOT_CONNECTED;


		for(int i = 0; i < str.length(); i++) {
			serialPort_->write(str[i]);
		}
		/*
		//cout<<"Writing: "<<ReplaceString(str, "\r", "\r\n");
		DWORD written = 0;
		int bStatus = WriteFile((HANDLE) handle, str.c_str(), str.length(), &written,  NULL);
		if (bStatus == 0)
		return RQ_ERR_TRANSMIT_FAILED;*/

		return RQ_SUCCESS;
	}
	int RoboteqDevice::Read(string &str, int count)
	{

		if(!IsConnected())
			return RQ_ERR_NOT_CONNECTED;

		str = "";
		circular_buffer<char> data = serialPort_->getDataRead();
		int start = data.size() - count;
		for (int i = 0; i < count; i++) {
			str.append(&data[ start + i ]);
		}


		return RQ_SUCCESS;
	}

	int RoboteqDevice::IssueCommand(string commandType, string command, string args, int waitms, string &response, bool isplusminus)
	{
		int status;
		string read;
		response = "";
		serialPort_->startReadCount();

		if(args == "")
			status = Write(commandType + command + "_");
		else
			status = Write(commandType + command + " " + args + "_");

		if(status != RQ_SUCCESS)
			return status;
		sleepms(waitms);
		int responseLength = serialPort_->getReadCount();


		status = Read(read,responseLength);
		if(status != RQ_SUCCESS)
			return status;

		if(isplusminus)
		{
			if(read.length() < 2)
				return RQ_INVALID_RESPONSE;

			response = read.substr(read.length() - 2, 1);
			return RQ_SUCCESS;
		}

		string::size_type pos = read.rfind(command + "=");
		if(pos == string::npos)
			return RQ_INVALID_RESPONSE;

		pos += command.length() + 1;

		string::size_type carriage = read.find("\r", pos);
		if(carriage == string::npos)
			return RQ_INVALID_RESPONSE;

		response = read.substr(pos, carriage - pos);

		return RQ_SUCCESS;
	}
	int RoboteqDevice::IssueCommand(string commandType, string command, int waitms, string &response, bool isplusminus)
	{
		return IssueCommand(commandType, command, "", waitms, response, isplusminus);
	}

	int RoboteqDevice::SetConfig(int configItem, int index, int value)
	{
		string response;
		char command[10];
		char args[50];

		if(configItem < 0 || configItem > 255)
			return RQ_INVALID_CONFIG_ITEM;

		sprintf(command, "$%02X", configItem);
		sprintf(args, "%i %i", index, value);
		if(index == MISSING_VALUE)
		{
			sprintf(args, "%i", value);
			index = 0;
		}

		if(index < 0)
			return RQ_INDEX_OUT_RANGE;

		int status = IssueCommand("^", command, args, 10, response, true);
		if(status != RQ_SUCCESS)
			return status;
		if(response != "+")
			return RQ_SET_CONFIG_FAILED;

		return RQ_SUCCESS;
	}
	int RoboteqDevice::SetConfig(int configItem, int value)
	{
		return SetConfig(configItem, MISSING_VALUE, value);
	}

	int RoboteqDevice::SetCommand(int commandItem, int index, int value)
	{
		string response;
		char command[10];
		char args[50];

		if(commandItem < 0 || commandItem > 255)
			return RQ_INVALID_COMMAND_ITEM;

		sprintf(command, "$%02X", commandItem);
		sprintf(args, "%i %i", index, value);
		if(index == MISSING_VALUE)
		{
			if(value != MISSING_VALUE)
				sprintf(args, "%i", value);
			index = 0;
		}

		if(index < 0)
			return RQ_INDEX_OUT_RANGE;

		int status = IssueCommand("!", command, args, 10, response, true);
		if(status != RQ_SUCCESS)
			return status;
		if(response != "+")
			return RQ_SET_COMMAND_FAILED;

		return RQ_SUCCESS;
	}
	int RoboteqDevice::SetCommand(int commandItem, int value)
	{
		return SetCommand(commandItem, MISSING_VALUE, value);
	}
	int RoboteqDevice::SetCommand(int commandItem)
	{
		return SetCommand(commandItem, MISSING_VALUE, MISSING_VALUE);
	}

	int RoboteqDevice::GetConfig(int configItem, int index, int &result)
	{
		string response;
		char command[10];
		char args[50];

		if(configItem < 0 || configItem > 255)
			return RQ_INVALID_CONFIG_ITEM;

		if(index < 0)
			return RQ_INDEX_OUT_RANGE;

		sprintf(command, "$%02X", configItem);
		sprintf(args, "%i", index);

		int status = IssueCommand("~", command, args, 10, response);
		if(status != RQ_SUCCESS)
			return status;

		istringstream iss(response);
		iss>>result;

		if(iss.fail())
			return RQ_GET_CONFIG_FAILED;

		return RQ_SUCCESS;
	}
	int RoboteqDevice::GetConfig(int configItem, int &result)
	{
		return GetConfig(configItem, 0, result);
	}

	int RoboteqDevice::GetValue(int operatingItem, int index, int &result)
	{
		string response;
		char command[10];
		char args[50];

		if(operatingItem < 0 || operatingItem > 255)
			return RQ_INVALID_OPER_ITEM;

		if(index < 0)
			return RQ_INDEX_OUT_RANGE;

		sprintf(command, "$%02X", operatingItem);
		sprintf(args, "%i", index);

		int status = IssueCommand("?", command, args, 10, response);
		if(status != RQ_SUCCESS)
			return status;

		istringstream iss(response);
		iss>>result;

		if(iss.fail())
			return RQ_GET_VALUE_FAILED;

		return RQ_SUCCESS;
	}
	int RoboteqDevice::GetValue(int operatingItem, int &result)
	{
		return GetValue(operatingItem, 0, result);
	}


	string ReplaceString(string source, string find, string replacement)
	{
		string::size_type pos = 0;
		while((pos = source.find(find, pos)) != string::npos)
		{
			source.replace(pos, find.size(), replacement);
			pos++;
		}

		return source;
	}

	void sleepms(int milliseconds)
	{
		Sleep(milliseconds);
	}

	bool RoboteqDevice::connectionTestOk() {
		serialPort_->write("?FID_");
		Sleep(500);

		circular_buffer<char> data = serialPort_->getDataRead();

		for(int i = data.size() - 1; i > 2; i--) {
			// looking for a string "FID="
			if((data[ i ] = '=') && (data [ i - 1 ] == 'D') && (data [ i - 2 ] == 'I') && (data [ i - 3 ] == 'F'))
				return true;
		}
		return false;
	}

	void RoboteqDevice::searchForDevice() {
		cout << "looking for Roboteq driver\n";
		std::vector<std::string> comList;
		serialPort_->DetectComPorts(comList, 128);

		for each (std::string port in comList) {
			serialPort_->open(baud_, port);
			if( connectionTestOk() )
				cout << "Roboteq driver connected!\n";
				return;
		}

	}

}