#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/filesystem.hpp>

#include <deque>
#include <iostream>
#include <string>
#include <cstdlib>
#include <dirent.h>
#include <cstdio>
#include <sys/types.h>

#include "../include/SerialPort.h"

using namespace std;
using namespace boost;

namespace trobot {
	SerialPort::SerialPort(unsigned int baud, const string& device) 
                : active_(true), 
                  io_service_(), 
                  //serialPort_(io_service_, device),
				  readBuffer_(max_read_length)
	{
		try {
			serialPort_ = new asio::serial_port(io_service_, device);
		}
		catch (...) {
			active_ = false;
			throw "Failed to open serial port";
		}
		if (!serialPort_->is_open()) 
		{
			cerr << "Failed to open serial port\n"; 
			throw "Failed to open serial port";
			return; 
		} 
		
		asio::serial_port_base::baud_rate baud_option(baud); 
		serialPort_->set_option(baud_option); // set the baud rate after the port has been opened 
		readStart();
		thread_ = new thread(bind(&asio::io_service::run, &io_service_));
		counting = false;
		newDataAvaialble_ = false;
		
	}

	SerialPort::~SerialPort(void)
	{
	}

	void SerialPort::test() {
		std::cout<<"test\n";
	}

	void SerialPort::readStart(void) {
		serialPort_->async_read_some(asio::buffer(read_msg_, max_read_length), 
									bind(&SerialPort::readComplete, 
										this, 
										asio::placeholders::error, 
										asio::placeholders::bytes_transferred)); 
	}


	void SerialPort::readComplete(const system::error_code& error, size_t bytes_transferred)
        { 
                if (!error) 
                { // read completed, so process the data 
                        //cout.write(read_msg_, bytes_transferred); // echo to standard output 

						
						bytesTransffered_ = (int)bytes_transferred;
						charCount += bytesTransffered_;
						//cout << bytesTransffered_ << endl;
						

						readBufferGuard_.lock();
						newDataAvaialble_ = true;
						for(int i = 0; i < bytesTransffered_; i++) {
							readBuffer_.push_back(read_msg_[ i ]);
							//cout << read_msg_[ i ];
				
						}	
						//cout <<endl;
						readBufferGuard_.unlock();

                        readStart(); // start waiting for another asynchronous read again 
                } 
                else 
                        doClose(error); 
        }

	void SerialPort::doWrite(const char msg) 
        { // callback to handle write call from outside this class 
                bool write_in_progress = !write_msgs_.empty(); // is there anything currently being written? 
                write_msgs_.push_back(msg); // store in write buffer 
                if (!write_in_progress) // if nothing is currently being written, then start 
                        writeStart(); 
	}

	void SerialPort::doClose(const system::error_code& error) 
        { // something has gone wrong, so close the socket & make this object inactive 
                if (error == asio::error::operation_aborted) // if this call is the result of a timer cancel() 
                        return; // ignore it because the connection cancelled the timer 
                if (error) 
                        cerr << "Error: " << error.message() << endl; // show the error message 
                //else 
                        //cout << "Error: Connection did not succeed.\n"; 
                //cout << "Press Enter to exit\n"; 
                serialPort_->close(); 
                active_ = false; 
        }

	void SerialPort::writeComplete(const boost::system::error_code& error) 
        { // the asynchronous read operation has now completed or failed and returned an error 
                if (!error) 
                { // write completed, so send next write data 
                        write_msgs_.pop_front(); // remove the completed data 
                        if (!write_msgs_.empty()) // if there is anthing left to be written 
                                writeStart(); // then start sending the next item in the buffer 
                } 
                else 
                        doClose(error); 
        }

	void SerialPort::writeStart(void) 
        { // Start an asynchronous write and call write_complete when it completes or fails 
                asio::async_write(*serialPort_, 
								asio::buffer(&write_msgs_.front(), 1), 
								bind(&SerialPort::writeComplete, 
									this, 
									asio::placeholders::error)); 
        }

	void SerialPort::write(const char msg) // pass the write data to the do_write function via the io service in the other thread 
        { 
			    
			io_service_.post(boost::bind(&SerialPort::doWrite, this, msg)); 
        } 
        
	void SerialPort::close() // call the do_close function via the io service in the other thread 
        { 
                io_service_.post(boost::bind(&SerialPort::doClose, this, boost::system::error_code())); 
				thread_->join();
        } 

	bool SerialPort::isActive() // return true if the socket is still active 
        { 
                return active_; 
        } 

	circular_buffer<char>  SerialPort::getDataRead() {
		readBufferGuard_.lock();
		circular_buffer<char> returnData = readBuffer_;
		newDataAvaialble_ = false;
		readBufferGuard_.unlock();
		return returnData;
	}

	bool SerialPort::newDataAvailable() {
		return newDataAvaialble_;
	}


	void SerialPort::startReadCount() {
		charCount = 0;
		counting = true;
	}

	int SerialPort::getReadCount() {
		counting = false;
		return charCount;
	}



	void SerialPort::DetectComPorts(std::vector< std::string >& ports, size_t upperLimit = 128)
	{
		/*if(windows) {
		for(int i = 0; i < upperLimit; i++) {
		asio::serial_port Serial;
		Serial.
		}*/
#if defined(WIN32) || defined(_WIN32)
		for(size_t i=1; i<=upperLimit; i++)
		{
			TCHAR sPort[32] = {0};
			_stprintf(sPort, _T("\\\\.\\COM%d"), i);

			BOOL bSuccess = FALSE;
			HANDLE hPort = ::CreateFile(sPort, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
			if (hPort == INVALID_HANDLE_VALUE)
			{
				DWORD dwError = GetLastError();

				//Check to see if the error was because some other app had the port open or a general failure
				if (dwError == ERROR_ACCESS_DENIED || dwError == ERROR_GEN_FAILURE || dwError == ERROR_SHARING_VIOLATION || dwError == ERROR_SEM_TIMEOUT)
					bSuccess = TRUE;
			}
			else
			{
				//The port was opened successfully
				bSuccess = TRUE;

				//Don't forget to close the port, since we are going to do nothing with it anyway
				CloseHandle(hPort);
			}

			//Add the port number to the array which will be returned
			if (bSuccess) {
				char result[32];
				wcstombs(result,sPort,sizeof(sPort));
				ports.push_back(result);
			}
		}
#else
		const filesystem::path dirPath("/dev");
		vector<string> names;
		names.push_back(string("ttyACM"));
		names.push_back(string("ttyUSB"));
		filesystem::directory_iterator endIt;
		for(filesystem::directory_iterator dirIt(dirPath); dirIt != endIt; dirIt++){
			for(int i = 0; i < names.size(); i++){
				if(string(dirIt->path().filename().c_str()).find(names[i]) != string::npos){
					ports.push_back(dirIt->path().c_str());
				}
			}
		}
#endif
	}
	

	void SerialPort::open(unsigned int baud, const std::string &device) {
		/*try{
			if(serialPort_ != NULL) {
				if(serialPort_->is_open()) {
				close();
				}
			}
		
		}
		catch(...) {
			;
		}*/
		serialPort_->open(device);
		if (!serialPort_->is_open()) 
		{
			cerr << "Failed to open serial port\n"; 
			throw "Failed to open serial port";
			return; 
		} 
		
		asio::serial_port_base::baud_rate baud_option(baud); 
		serialPort_->set_option(baud_option); // set the baud rate after the port has been opened 
		readStart();
		thread_ = new thread(bind(&asio::io_service::run, &io_service_));
		counting = false;
	}

	void SerialPort::write(const std::string &msg) {
		//cout << "write: "<< msg <<endl;
		for(int i = 0; i < msg.length(); i++) {
			write((char)msg[ i ]);
		}
	}

	int searchBufferR(const circular_buffer<char> & buffer, const std::string & text) {
		int buff_size = buffer.size();
		int length = text.length();
		if(length > buff_size)
			return -1;
		for(int i = buff_size - length; i >= 0; i--) {
			bool testOk = true;
			for(int j = 0; j < length; j++) {
				if(buffer[ i + j ] != text[ j ]) {
					testOk = false;
					break;
				}
			}
			if(testOk)
				return i;
		}
		return -1;
	}
	
	
	std::wstring SerialPort::s2ws(const string& s){
		int len;
		wchar_t buf[MAX_WSTR_LEN];
		mbstowcs(buf, s.c_str(), MAX_WSTR_LEN);
		std::wstring r(buf);
		return r;
	}
		
	void SerialPort::flush() {
		if(readBuffer_.size() != 0)
			readBuffer_.clear();
	}
};

