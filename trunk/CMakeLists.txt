cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CUDA 1)
set(CMAKE_BUILD_TYPE "Debug")

project(RobotsIntellect)

SET(CMAKE_CXX_FLAGS "-std=c++0x -DUSE_USB_CONNECTION")

IF (CUDA EQUAL 0)
	add_definitions(-DNO_CUDA)
ENDIF (CUDA EQUAL 0)

set(urg_LIBRARIES_DIR /usr/local/lib)
set(urg_LIBRARIES urg_cpp)
set(TinyXML_LIBRARIES tinyxml)

find_package(OpenCV REQUIRED)

find_package(Boost COMPONENTS system filesystem thread REQUIRED)

IF (CUDA GREATER 0)
	find_package(CUDA REQUIRED)
	# Pass options to NVCC
	set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -gencode arch=compute_20,code=sm_21)
	SET( CUDA_PROPAGATE_HOST_FLAGS OFF )
ENDIF (CUDA GREATER 0)
link_directories(${URG_LIBRARIES_DIR})


include_directories(MicrostrainSDK/Library/Include)
include_directories(MicrostrainSDK/Utilities/Include)
include_directories(MicrostrainSDK/Library/User\ Functions)
set(MicrostrainSDK_SOURCES
	MicrostrainSDK/Library/Source/mip_sdk_system.c 
	MicrostrainSDK/Library/Source/mip_sdk_base.c 
	MicrostrainSDK/Library/Source/mip_sdk_gps.c 
	MicrostrainSDK/Library/Source/mip.c 
    MicrostrainSDK/Library/Source/mip_sdk_3dm.c 
	MicrostrainSDK/Library/Source/ring_buffer.c 
	MicrostrainSDK/Library/Source/mip_sdk_inteface.c 
	MicrostrainSDK/Library/Source/mip_sdk_filter.c
    MicrostrainSDK/Library/Source/mip_sdk_ahrs.c
	MicrostrainSDK/Library/Include/mainpage.h    
	MicrostrainSDK/Library/Include/mip_gx4_imu.h   
	MicrostrainSDK/Library/Include/mip_sdk_base.h       
	MicrostrainSDK/Library/Include/mip_sdk_system.h
	MicrostrainSDK/Library/Include/mip_gx3_35.h  
	MicrostrainSDK/Library/Include/mip.h           
	MicrostrainSDK/Library/Include/mip_sdk_config.h     
	MicrostrainSDK/Library/Include/mip_types.h
	MicrostrainSDK/Library/Include/mip_gx3_45.h  
	MicrostrainSDK/Library/Include/mip_rq1.h       
	MicrostrainSDK/Library/Include/mip_sdk_filter.h     
	MicrostrainSDK/Library/Include/ring_buffer.h
	MicrostrainSDK/Library/Include/mip_gx4_15.h  
	MicrostrainSDK/Library/Include/mip_rq1_imu.h   
	MicrostrainSDK/Library/Include/mip_sdk_gps.h
	MicrostrainSDK/Library/Include/mip_gx4_25.h  
	MicrostrainSDK/Library/Include/mip_sdk_3dm.h   
	MicrostrainSDK/Library/Include/mip_sdk.h
	MicrostrainSDK/Library/Include/mip_gx4_45.h  
	MicrostrainSDK/Library/Include/mip_sdk_ahrs.h  
	MicrostrainSDK/Library/Include/mip_sdk_interface.h
	MicrostrainSDK/Library/User\ Functions/mip_sdk_user_functions.c
	MicrostrainSDK/Library/User\ Functions/mip_sdk_user_functions.h
)


set(MicrostrainSDK_UTIL_SOURCES
	MicrostrainSDK/Utilities/Source/byteswap_utilities.c)


set(RobotsIntellect_SOURCES
	${MicrostrainSDK_SOURCES}
	${MicrostrainSDK_UTIL_SOURCES}
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
	Planning/LocalPlanner.cpp
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
	PositionEstimation/IMU/src/IMU_driver.cpp
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

IF( CUDA GREATER 0)
	set(RobotsIntellect_CUDA_SOURCES
		MovementConstraints/Camera/Camera.cu
	)
	cuda_add_executable (RobotsIntellect 
		${RobotsIntellect_SOURCES}
		${RobotsIntellect_CUDA_SOURCES}
		main.cpp
	)
ELSEIF ( CUDA EQUAL 0)
	add_executable (RobotsIntellect 
		${RobotsIntellect_SOURCES}
		main.cpp
	)
ENDIF( CUDA GREATER 0)

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

IF (CUDA GREATER 0)
	cuda_add_executable(Trobot_Qt/gui
		${RobotsIntellect_SOURCES}
		${RobotsIntellect_CUDA_SOURCES}
		${trobotQt_SOURCES}
		${trobotQt_HEADERS_MOC}
		${trobotQt_FORMS_HEADERS})
ELSEIF (CUDA EQUAL 0)
	add_executable(Trobot_Qt/gui
		${RobotsIntellect_SOURCES}
		${trobotQt_SOURCES}
		${trobotQt_HEADERS_MOC}
		${trobotQt_FORMS_HEADERS})
ENDIF (CUDA GREATER 0)	

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
