#ifndef ROBOT_DRIVE_H
#define ROBOT_DRIVE_H

#include "../include/RoboteqDevice.h"
#include <string>

#define STOP 0
#define POSITION 1
#define SPEED 2
#define TURN 3
#define NO_PARAM -1001
#define UNDEFINED -99999999

namespace trobot {
	struct PidParams{ ///< divide all params by 10 to obtain real value
		int kp;
		int kd;
		int ki;
	};

	class RobotDrive
	{
	public:
		RobotDrive(const std::string& device, unsigned int baud = 115200);
	public:
		~RobotDrive(void);

		void	driveStraight(int speed);					///< drive for indefinite period of time. speed in 1/10 percent, positive forward, negative backward
		//void	driveStraight(int speed, int time);			///< time given in 1/10 of second
		void	driveStraight(int speed, float distance);	///< distance in meters, maximum 20m. Throws exception if distance is too big
		void	stop(void);									///< stop immidiately
		void	turn(int speed, int degrees);				///< turns the given angle, positive - clockwise
		void	runMotor(int speed, int channel);			///< runs a motor connected to a specific channel
		bool	positionReached();							///< checks whether robot reached desired position. Also returns true if the robot was in the other mode than position. It is assumed that position is reached, when speed of the robot is 0.
		void	resume(void);								///< resumes motion of a robot after obstacle was removed.


	private:
		
		float				distanceBetweenWheels;	///< in meters
		float				wheelDiameter;			///< in meters
		int					encoderCPR;				///< measuered for a wheel revolution
		int					maxRpms;				///< measured for a wheel
		int					lastCommand;			///< stores last command issued to a driver
		PidParams			positionPid;			
		PidParams			speedPid;

		int					lastSpeed;				///< last movement speed
		int					lastGoalL;				///< last desired position of left wheel
		int					lastGoalR;				///< last desired position of right wheel

		int					mode_;					///< controller mode - POSITION or SPEED

		SerialPort *		serialPort_;
		unsigned int		baud_;
		std::string				device_;

		bool				posCheckedFlag_;		///< used to prevent one-time wrong reading errors

		void				searchForDevice();	///< searches for a device and connects to it
		bool				connectionTestOk(); ///< tests if the connection to the driver was properly established

		

		void				initConfig(void);		///< loads initial configuration into driver

		void				setRuntime(std::string command, int param1, int param2); ///< sends runtime commands

		void				getRuntime(const std::string &command, int param, int *result);
		void				getRuntime(const std::string &command, int *result1, int *result2);
		void				getRuntime(const std::string &command, int param, int *result1, int *result2);

		void				setConfig(std::string command, int param1, int param2);
		void				getConfig(std::string command, int param1, int param2);


		void				setSpeedMode();
		void				setPositionMode();



	};
}

#endif //ROBOT_DRIVE_H

