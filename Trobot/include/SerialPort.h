#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <boost/bind.hpp>
#include <boost/asio.hpp> 
#include <boost/asio/serial_port.hpp> 
#include <boost/thread.hpp> 
#include <boost/lexical_cast.hpp> 
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/circular_buffer.hpp>

#include <deque>
#include <iostream>
#include <string>
#include <cstdlib>

#if defined(WIN32) || defined(_WIN32)

#include <windows.h>
#include <TCHAR.H>

#else

#define MAX_WSTR_LEN 200

#endif


namespace trobot {
	int searchBufferR(const boost::circular_buffer<char> & buffer, const std::string & text); ///< search for last occurence of specified string in buffer. Returns position of first character of the string. If negative, no occurances were found

	class SerialPort
	{
	public:
		SerialPort();
		SerialPort(unsigned int baud, const std::string& device);

		void			test();
		void			write(const char msg);
		void			write(const std::string& msg);
		void			close();
		void			flush();
		bool			isActive();

		bool			newDataAvailable();
		boost::circular_buffer<char> 	getDataRead(); ///< returns data from the buffer

		void			startReadCount();	///< starts counting incoming characters
		int				getReadCount();		///< stops counting incoming characters and returns its result
		
		void			open(unsigned int baud, const std::string& device);
		static void		DetectComPorts(std::vector< std::string >& ports, size_t upperLimit); ///< this works only on windows, detects COM ports, assumes that no new ports are added during runtime
		static std::wstring		s2ws(const std::string& s);
	public:
		~SerialPort(void);
		
		
		
		static const int	max_read_length = 512; // maximum amount of data to read in one operation
	private:
		boost::mutex		readBufferGuard_; ///<prevents read buffer to be read and modified by multiple threads simultaneously

		unsigned int		bytesTransffered_;

		boost::asio::io_service	io_service_; ///< the main IO service that runs this connection
		boost::asio::io_service::work io_service_work;	///< Ensures that service's run() won't exit when there's nothing to do
		boost::thread				thread_; ///< thread that runs the boost serial port
        boost::asio::serial_port	serialPort_; ///< the serial port this instance is connected to
		static std::vector<std::string> availablePorts; ///< stores list of all available COM ports

		volatile bool				active_; ///< remains true while this object is still operating
		volatile bool				newDataAvaialble_;
        
		char					read_msg_[max_read_length]; // data read from the socket 
		boost::circular_buffer<char>	readBuffer_; ///< nice structure to store data from boost library
		std::deque<char>				write_msgs_; // buffered write data




		
		
		void		readStart(void); ///< Start an asynchronous read and call read_complete when it completes or fails 
		void		readComplete(const boost::system::error_code& error, size_t bytes_transferred); ///< the asynchronous read operation has now completed or failed and returned an error 

		void		doWrite(const char msg); ///< callback to handle write call from outside this class
		void		writeStart(void); ///< Start an asynchronous write and call write_complete when it completes or fails 
		void		writeComplete(const boost::system::error_code& error); ///< the asynchronous read operation has now completed or failed and returned an error 

		void		doClose(const boost::system::error_code& error); ///< something has gone wrong, so close the socket & make this object inactive 

		int			charCount; ///< used to count incoming characters
		bool		counting; ///< indicates that we are currently counting incoming chars
	};
}

#endif //SERIAL_PORT_H

