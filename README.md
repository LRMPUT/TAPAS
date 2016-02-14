# TAPAS
Code for an autonomous mobile robot TAPAS developed at Poznan University of Technology.

Prerequesities:
-VTK >= 5.0
-OpenCV >= 3.0 with viz module
-Qt 4.x
-QGLViewer
-TinyXML
-URG library
-libLBFGS
-optionally CUDA >= 5.0

Prerequesities installation:
1. Install prerequesities with apt-get: sudo apt-get install libvtk5-dev liblbfgs-dev libtinyxml-dev libqglviewer-dev
2. Optionally: sudo apt-get install nvidia-cuda-toolkit (a proper driver have to be installed already)
3. Compile and install OpenCV 3.0 with viz module.
4. Compile and install URG library: http://sourceforge.net/projects/urgnetwork/files/urg_library/ .


Build:
mkdir build
cd build
(adjust CUDA flag in CMakeLists.txt)
cmake ..
make
