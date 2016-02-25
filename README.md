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

###Prerequesities installation:  
1. Install prerequesities with apt-get:
```
sudo apt-get install libvtk5-dev liblbfgs-dev libtinyxml-dev libqglviewer-dev
```
2. Optionally for a CUDA support (a proper driver has to be installed already):
```
sudo apt-get install nvidia-cuda-toolkit
```
3. Compile and install OpenCV 3.1 with viz module:
```
wget https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/3.1.0/opencv-3.1.0.zip
unzip opencv-3.1.0.zip && cd opencv-3.1.0
mkdir build && cd build
cmake -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)
sudo make install
```
4. Compile and install URG library:  
```
wget https://sourceforge.net/projects/urgnetwork/files/urg_library/urg_library-1.2.0.zip
unzip urg_library-1.2.0.zip && cd urg_library-1.2.0
make -j$(nproc)
sudo make install
```

###Build:
```
mkdir build
cd build
#(adjust CUDA_NVCC_FLAGS flags in CMakeLists.txt to a proper arch and code)
cmake ..
make -j$(nproc)
```
