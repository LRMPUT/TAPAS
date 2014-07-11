cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_BUILD_TYPE "Debug")

project(RobotsIntellect)

SET(CMAKE_CXX_FLAGS "-std=c++0x")

set(urg_LIBRARIES_DIR /usr/local/lib)
set(urg_LIBRARIES urg_cpp)
set(TinyXML_LIBRARIES tinyxml)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system filesystem thread REQUIRED)
link_directories(${URG_LIBRARIES_DIR})
set(RobotsIntellect_SOURCES
	Debug/Debug.cpp
	MechanicalGrabber/MechanicalGrabber.cpp
	MovementConstraints/Hokuyo/Hokuyo.cpp
	MovementConstraints/Sharp/Sharp.cpp
	MovementConstraints/Camera/LibSVM-weights/svm.cpp
	MovementConstraints/Camera/Camera.cpp
	MovementConstraints/Camera/HierClassifier/UnionFind.cpp
	MovementConstraints/Camera/HierClassifier/Classifier.cpp
	MovementConstraints/Camera/HierClassifier/ClassifierSVM.cpp
	MovementConstraints/Camera/HierClassifier/HierClassifier.cpp
	MovementConstraints/MovementConstraints.cpp
	#Planning/PlanningSearching/LocalPlannerSearching.cpp
	#Planning/PlanningToGold/LocalPlannerToGold.cpp
	#Planning/PlanningWithGold/LocalPlannerWithGold.cpp
	Planning/RobotDrivers/robotDrivers.cpp
	Planning/GlobalPlanner.cpp
	PositionEstimation/Util/ExtendedKalmanFilter.cpp
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
	Trobot/source/rangeSensor.cpp
	Trobot/source/RobotDrive.cpp
	Trobot/source/RoboteqDevice.cpp
	Trobot/source/SerialPort.cpp
)

add_executable (RobotsIntellect 
	${RobotsIntellect_SOURCES}
	main.cpp
)

target_link_libraries(RobotsIntellect
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	${urg_LIBRARIES}
	${TinyXML_LIBRARIES})

find_package(OpenGL REQUIRED)
set(QGLViewer_LIBRARIES QGLViewer)

find_package(Qt4 COMPONENTS QtCore QtXml QtGui QtOpenGL REQUIRED)
set(trobotQt_SOURCES
	Trobot_Qt/CameraWindow.cpp
	Trobot_Qt/Chart.cpp
	Trobot_Qt/gui.cpp
	Trobot_Qt/ImuChart.cpp
	Trobot_Qt/QtRobotDrive.cpp
	Trobot_Qt/Recording.cpp
	Trobot_Qt/QtGps.cpp
	Trobot_Qt/QtCamera.cpp
	Trobot_Qt/QtHokuyo.cpp
	Trobot_Qt/Calibration.cpp
	Trobot_Qt/Constraints.cpp
	Trobot_Qt/Viewer.cpp
	Trobot_Qt/trobotqt.cpp
	Trobot_Qt/QtEncoders.cpp)
set(trobotQt_HEADERS
	Trobot_Qt/CameraWindow.h
	Trobot_Qt/Chart.h
	Trobot_Qt/ImuChart.h
	Trobot_Qt/QtRobotDrive.h
	Trobot_Qt/Recording.h
	Trobot_Qt/QtGps.h
	Trobot_Qt/QtCamera.h
	Trobot_Qt/QtHokuyo.h
	Trobot_Qt/Calibration.h
	Trobot_Qt/Constraints.h
	Trobot_Qt/Viewer.h
	Trobot_Qt/trobotqt.h
	Trobot_Qt/QtEncoders.h)
set(trobotQt_FORMS
	Trobot_Qt/CameraWindow.ui
	Trobot_Qt/trobotqt.ui)
qt4_wrap_cpp(trobotQt_HEADERS_MOC ${trobotQt_HEADERS} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
qt4_wrap_ui(trobotQt_FORMS_HEADERS ${trobotQt_FORMS})
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})
add_executable(Trobot_Qt/gui
	${RobotsIntellect_SOURCES}
	${trobotQt_SOURCES}
	${trobotQt_HEADERS_MOC}
	${trobotQt_FORMS_HEADERS})
message("Copying generated ui headers")
foreach(file ${trobotQt_FORMS_HEADERS})
	get_filename_component(filename ${file} NAME)
	message("Copying: " ${filename})
	add_custom_command(
		OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/Trobot_Qt/${filename}
		COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/${filename} ${CMAKE_CURRENT_SOURCE_DIR}/Trobot_Qt/${filename}
		MAIN_DEPENDENCY ${CMAKE_CURRENT_BINARY_DIR}/${filename})
endforeach()
target_link_libraries(Trobot_Qt/gui
	${QT_LIBRARIES}
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	${OPENGL_LIBRARIES}
	${QGLViewer_LIBRARIES}
	${urg_LIBRARIES}
	${TinyXML_LIBRARIES})
