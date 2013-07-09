cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(RobotsIntellect)

set(urg_LIBRARIES_DIR /usr/local/lib)
set(urg_LIBRARIES urg_c)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system filesystem thread REQUIRED)
include_directories(Trobot/include)
link_directories(${URG_LIBRARIES_DIR})
set(RobotsIntellect_SOURCES
	MechanicalGrabber/MechanicalGrabber.cpp
	MovementConstraints/Hokuyo/Hokuyo.cpp
	MovementConstraints/Sharp/Sharp.cpp
	MovementConstraints/Camera/Camera.cpp
	MovementConstraints/MovementConstraints.cpp
	#Planning/PlanningSearching/LocalPlannerSearching.cpp
	#Planning/PlanningToGold/LocalPlannerToGold.cpp
	#Planning/PlanningWithGold/LocalPlannerWithGold.cpp
	Planning/GlobalPlanner.cpp
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
)
add_executable (RobotsIntellect 
	${RobotsIntellect_SOURCES}
	main.cpp
)
target_link_libraries(RobotsIntellect ${OpenCV_LIBS} ${Boost_LIBRARIES} ${urg_LIBRARIES})

find_package(Qt4 REQUIRED)
set(trobotQt_SOURCES
	Trobot_Qt/CameraWindow.cpp
	Trobot_Qt/Chart.cpp
	Trobot_Qt/gui.cpp
	Trobot_Qt/ImuChart.cpp
	Trobot_Qt/QtRobotDrive.cpp
	Trobot_Qt/Recording.cpp
	Trobot_Qt/trobotqt.cpp)
set(trobotQt_HEADERS
	Trobot_Qt/CameraWindow.h
	Trobot_Qt/Chart.h
	Trobot_Qt/ImuChart.h
	Trobot_Qt/QtRobotDrive.h
	Trobot_Qt/Recording.h
	Trobot_Qt/trobotqt.h)
set(trobotQt_FORMS
	Trobot_Qt/CameraWindow.ui
	Trobot_Qt/trobotqt.ui)
qt4_wrap_cpp(trobotQt_HEADERS_MOC ${trobotQt_HEADERS})
qt4_wrap_ui(trobotQt_FORMS_HEADERS ${trobotQt_FORMS})
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
add_executable(Trobot_Qt/gui
	${RobotsIntellect_SOURCES}
	${trobotQt_SOURCES}
	${trobotQt_HEADERS_MOC}
	${trobotQt_FORMS_HEADERS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(Trobot_Qt/gui
	${QT_LIBRARIES}
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	${urg_LIBRARIES})
