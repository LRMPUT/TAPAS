cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

SET(CMAKE_CXX_FLAGS "-std=c++0x")
add_definitions( -DBOOST_ALL_NO_LIB )
set( Boost_USE_DYNAMIC_LIBS ON )

project(RobotsIntellect)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)

include_directories( ${Boost_INCLUDE_DIR} )
include_directories( ${CMAKE_SOURCE_DIR} )


add_executable (RobotsIntellect
MechanicalGrabber/MechanicalGrabber.cpp
MovementConstraints/Hokuyo/Hokuyo.cpp
MovementConstraints/Sharp/Sharp.cpp
MovementConstraints/MovementConstraints.cpp
Planning/GlobalPlanner.cpp
Planning/PlanningSearching/LocalPlannerSearching.cpp
Planning/PlanningToGold/LocalPlannerToGold.cpp
Planning/PlanningWithGold/LocalPlannerWithGold.cpp
PositionEstimation/Encoders/Encoders.cpp
PositionEstimation/GPS/src/context.c
PositionEstimation/GPS/src/generate.c
PositionEstimation/GPS/src/generator.c
PositionEstimation/GPS/src/gmath.c
PositionEstimation/GPS/src/info.c
PositionEstimation/GPS/src/parse.c
PositionEstimation/GPS/src/parser.c
PositionEstimation/GPS/src/sentence.c
PositionEstimation/GPS/src/time.c
PositionEstimation/GPS/src/tok.c
PositionEstimation/GPS/GPS.cpp
PositionEstimation/IMU/IMU.cpp
PositionEstimation/PositionEstimation.cpp
Robot/Robot.cpp
Trobot/source/Controller.cpp
Trobot/source/Gps.cpp
Trobot/source/Imu.cpp
Trobot/source/Motor.cpp
Trobot/source/rangeSensor.cpp
Trobot/source/RobotDrive.cpp
Trobot/source/RoboteqDevice.cpp
Trobot/source/SerialPort.cpp
main.cpp )


target_link_libraries (RobotsIntellect ${OpenCV_LIBS} ${Boost_LIBRARIES} ${cmaketest_HEADERS})


	
