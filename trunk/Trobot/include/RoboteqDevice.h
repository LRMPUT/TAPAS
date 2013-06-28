#ifndef __RoboteqDevice_H_
#define __RoboteqDevice_H_

#include "SerialPort.h"
#include "Constants.h"
#include <string>

namespace trobot {

std::string ReplaceString(std::string source, std::string find, std::string replacement);
void sleepms(int milliseconds);

class RoboteqDevice
{
private:
	int device_fd;
	int fd0;
	//int handle;
	SerialPort *		serialPort_;
	unsigned int		baud_;

	void				searchForDevice();	///< searches for a device and connects to it
	bool				connectionTestOk(); ///< tests if the connection to the driver was properly established

protected:
	void InitPort();

	
	int Read(std::string &str, int count);

	int IssueCommand(std::string commandType, std::string command, std::string args, int waitms, std::string &response, bool isplusminus = false);
	int IssueCommand(std::string commandType, std::string command, int waitms, std::string &response, bool isplusminus = false);

public:
	int Write(std::string str);
	bool IsConnected();
	int Connect(std::string port);
	void Disconnect();

	int SetConfig(int configItem, int index, int value);
	int SetConfig(int configItem, int value);

	int SetCommand(int commandItem, int index, int value);
	int SetCommand(int commandItem, int value);
	int SetCommand(int commandItem);

	int GetConfig(int configItem, int index, int &result);
	int GetConfig(int configItem, int &result);

	int GetValue(int operatingItem, int index, int &result);
	int GetValue(int operatingItem, int &result);

	RoboteqDevice(const std::string& device, unsigned int baud);
	~RoboteqDevice();
};

}

#endif
