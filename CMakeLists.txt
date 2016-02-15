<<<<<<< HEAD
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# 0 - without CUDA
# 1 - with CUDA
set(CUDA 1)
=======
cmake_minimum_required(VERSION 2.8.3)
project(TAPAS)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_service_files(
  FILES
  PointCloud.srv
  CameraConstraints.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES TAPAS
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 0 - without CUDA
# 1 - with CUDA
set(CUDA 0)
>>>>>>> master

# 0 - TAPAS
# 1 - TROBOT
set(TROBOT 0)

# Debug
# Release
set(CMAKE_BUILD_TYPE "Release")

project(Tapas)

SET(CMAKE_CXX_FLAGS "-std=c++0x -DUSE_USB_CONNECTION")

IF (CUDA EQUAL 0)
	add_definitions(-DNO_CUDA)
ENDIF (CUDA EQUAL 0)

IF (TROBOT GREATER 0)
	add_definitions(-DTROBOT)
ENDIF (TROBOT GREATER 0)

# Standard library directory when manual installed
set(urg_LIBRARIES_DIR /usr/local/lib)
set(urg_LIBRARIES urg_cpp)
set(TinyXML_LIBRARIES tinyxml)

find_package(OpenCV REQUIRED)

find_package(Boost COMPONENTS system filesystem thread REQUIRED)

set(LBFGS_LIBRARIES lbfgs)

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


set(Tapas_SOURCES
	${MicrostrainSDK_SOURCES}
	${MicrostrainSDK_UTIL_SOURCES}
	Debug/Debug.cpp
	MechanicalGrabber/MechanicalGrabber.cpp
	MovementConstraints/Hokuyo/Hokuyo.cpp
	MovementConstraints/Sharp/Sharp.cpp
	MovementConstraints/Camera/LibSVM-weights/svm.cpp
	MovementConstraints/Camera/Pgm/CustFeature.cpp
	MovementConstraints/Camera/Pgm/Inference.cpp
	MovementConstraints/Camera/Pgm/ParamEst.cpp
	MovementConstraints/Camera/Pgm/Pgm.cpp
	#MovementConstraints/Camera/Pgm/UnionFind.cpp
	MovementConstraints/Camera/Camera.cpp
	MovementConstraints/Camera/HierClassifier/UnionFind.cpp
	MovementConstraints/Camera/HierClassifier/Classifier.cpp
	MovementConstraints/Camera/HierClassifier/ClassifierSVM.cpp
	MovementConstraints/Camera/HierClassifier/ClassifierRF.cpp
	MovementConstraints/Camera/HierClassifier/HierClassifier.cpp
<<<<<<< HEAD
=======
	MovementConstraints/ConstraintsHelpers.cpp
>>>>>>> master
	MovementConstraints/MovementConstraints.cpp
	#Planning/PlanningSearching/LocalPlannerSearching.cpp
	#Planning/PlanningToGold/LocalPlannerToGold.cpp
	#Planning/PlanningWithGold/LocalPlannerWithGold.cpp
	Planning/RobotDrivers/robotDrivers.cpp
	Planning/GlobalPlanner.cpp
	Planning/LocalPlanner.cpp
<<<<<<< HEAD
=======
	Planning/PlannerHelpers.cpp
>>>>>>> master
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
	Trobot/source/SerialPort.cpp
)

IF(TROBOT GREATER 0)
	set(Tapas_SOURCES
		${Tapas_SOURCES}
		Trobot/source/Controller.cpp
		Trobot/source/Gps.cpp
		Trobot/source/Imu.cpp
		Trobot/source/rangeSensor.cpp
		Trobot/source/RobotDrive.cpp
		Trobot/source/RoboteqDevice.cpp
	)
ENDIF(TROBOT GREATER 0)

IF( CUDA GREATER 0)
	set(Tapas_CUDA_SOURCES
		MovementConstraints/Camera/Camera.cu
	)
	cuda_add_executable (Tapas 
		${Tapas_SOURCES}
		${Tapas_CUDA_SOURCES}
		main.cpp
	)
ELSEIF ( CUDA EQUAL 0)
	add_executable (Tapas 
		${Tapas_SOURCES}
		main.cpp
	)
ENDIF( CUDA GREATER 0)

target_link_libraries(Tapas
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	${urg_LIBRARIES}
	${TinyXML_LIBRARIES}
<<<<<<< HEAD
	${LBFGS_LIBRARIES})
=======
	${LBFGS_LIBRARIES}
	${catkin_LIBRARIES})
>>>>>>> master

find_package(OpenGL REQUIRED)
set(QGLViewer_LIBRARIES QGLViewer)

find_package(Qt4 COMPONENTS QtCore QtXml QtGui QtOpenGL REQUIRED)

set(TapasQt_SOURCES
	TapasQt/CameraWindow.cpp
	TapasQt/Chart.cpp
	TapasQt/gui.cpp
#	TapasQt/ImuChart.cpp
	TapasQt/QtRobotDrive.cpp
	TapasQt/Recording.cpp
	TapasQt/QtGps.cpp
	TapasQt/QtCamera.cpp
	TapasQt/QtHokuyo.cpp
	TapasQt/Calibration.cpp
	TapasQt/Constraints.cpp
	TapasQt/Viewer.cpp
	TapasQt/TapasQt.cpp
	TapasQt/QtEncoders.cpp)
	
set(TapasQt_HEADERS
	TapasQt/CameraWindow.h
	TapasQt/Chart.h
#	TapasQt/ImuChart.h
	TapasQt/QtRobotDrive.h
	TapasQt/Recording.h
	TapasQt/QtGps.h
	TapasQt/QtCamera.h
	TapasQt/QtHokuyo.h
	TapasQt/Calibration.h
	TapasQt/Constraints.h
	TapasQt/Viewer.h
	TapasQt/TapasQt.h
	TapasQt/QtEncoders.h)
	
set(TapasQt_FORMS
	TapasQt/CameraWindow.ui
	TapasQt/TapasQt.ui)
	
qt4_wrap_cpp(TapasQt_HEADERS_MOC ${TapasQt_HEADERS} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
qt4_wrap_ui(TapasQt_FORMS_HEADERS ${TapasQt_FORMS})
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

IF (CUDA GREATER 0)
	cuda_add_executable(TapasGui
		${Tapas_SOURCES}
		${Tapas_CUDA_SOURCES}
		${TapasQt_SOURCES}
		${TapasQt_HEADERS_MOC}
		${TapasQt_FORMS_HEADERS})
ELSEIF (CUDA EQUAL 0)
	add_executable(TapasGui
		${Tapas_SOURCES}
		${TapasQt_SOURCES}
		${TapasQt_HEADERS_MOC}
		${TapasQt_FORMS_HEADERS})
ENDIF (CUDA GREATER 0)	

<<<<<<< HEAD
=======
target_link_libraries(TapasGui
	${QT_LIBRARIES}
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	${OPENGL_LIBRARIES}
	${QGLViewer_LIBRARIES}
	${urg_LIBRARIES}
	${TinyXML_LIBRARIES}
	${LBFGS_LIBRARIES}
	${catkin_LIBRARIES})

set(Camera_SOURCES
	MovementConstraints/Camera/LibSVM-weights/svm.cpp
	MovementConstraints/Camera/Pgm/CustFeature.cpp
	MovementConstraints/Camera/Pgm/Inference.cpp
	MovementConstraints/Camera/Pgm/ParamEst.cpp
	MovementConstraints/Camera/Pgm/Pgm.cpp
	#MovementConstraints/Camera/Pgm/UnionFind.cpp
	MovementConstraints/Camera/Camera.cpp
	MovementConstraints/Camera/HierClassifier/UnionFind.cpp
	MovementConstraints/Camera/HierClassifier/Classifier.cpp
	MovementConstraints/Camera/HierClassifier/ClassifierSVM.cpp
	MovementConstraints/Camera/HierClassifier/ClassifierRF.cpp
	MovementConstraints/Camera/HierClassifier/HierClassifier.cpp
	MovementConstraints/ConstraintsHelpers.cpp
	Planning/PlannerHelpers.cpp)

IF( CUDA GREATER 0)
	cuda_add_executable(Camera
		${Camera_SOURCES}
		${Tapas_CUDA_SOURCES}
		MovementConstraints/Camera/main.cpp
	)
ELSEIF ( CUDA EQUAL 0)
	add_executable(Camera 
		${Camera_SOURCES}
		MovementConstraints/Camera/main.cpp
	)
ENDIF( CUDA GREATER 0)
>>>>>>> master

#message("Copying generated ui headers")
# Adding command for automatically copying generated ui headers 
#foreach(file ${TapasQt_FORMS_HEADERS})
#	get_filename_component(filename ${file} NAME)
#	message("Copying: " ${filename})
#	add_custom_command(
#		OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/TapasQt/${filename}
#		COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/${filename} ${CMAKE_CURRENT_SOURCE_DIR}/TapasQt/${filename}
#		MAIN_DEPENDENCY ${CMAKE_CURRENT_BINARY_DIR}/${filename})
#endforeach()

<<<<<<< HEAD
target_link_libraries(TapasGui
	${QT_LIBRARIES}
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	${OPENGL_LIBRARIES}
	${QGLViewer_LIBRARIES}
	${urg_LIBRARIES}
	${TinyXML_LIBRARIES}
	${LBFGS_LIBRARIES})
=======
target_link_libraries(Camera
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	${TinyXML_LIBRARIES}
	${LBFGS_LIBRARIES}
	${catkin_LIBRARIES})
>>>>>>> master
