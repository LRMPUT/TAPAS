/*
 * CameraUtils.cu
 *
 *  Created on: Jul 30, 2014
 *      Author: robots
 */
//#include <opencv2/opencv.hpp>

#include <cuda_runtime.h>
//#include <cutil.h>
//#include <helper_cuda.h>
//#include <helper_functions.h>
//#include <vector_types.h>

#include <cstdio>
#include <iostream>

// This will output the proper CUDA error strings in the event that a CUDA host call returns an error
#define checkCudaErrors(err)  __checkCudaErrors (err, __FILE__, __LINE__)

inline void __checkCudaErrors(cudaError err, const char *file, const int line )
{
    if(cudaSuccess != err)
    {
        fprintf(stderr, "%s(%i) : CUDA Runtime API error %d: %s.\n",file, line, (int)err, cudaGetErrorString( err ) );
        exit(-1);
    }
}

#include "CameraKernels.cu"

void cudaAllocateAndCopyToDevice(void** d_dst, void* src, int size){
	checkCudaErrors(cudaMalloc(d_dst, size));
	//std::cout << "*d_dst = " << *d_dst << ", src = " << src << ", size = " << size << std::endl;
	checkCudaErrors(cudaMemcpy(*d_dst, src, size, cudaMemcpyHostToDevice));
}

void cudaCopyFromDeviceAndFree(void* dst, void* d_src, int size){
	checkCudaErrors(cudaMemcpy(dst, d_src, size, cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaFree(d_src));
}

extern "C" void reprojectCameraPoints(float* invCameraMatrix,
							float* distCoeffs,
							float* curPosCameraMapCenterGlobal,
							float* curPosCameraMapCenterImu,
							int numRows,
							int numCols,
							int* segments,
							int mapSize,
							int rasterSize)
{
	float* d_invCameraMatrix;
	float* d_distCoeffs;
	float* d_curPosCameraMapCenterGlobal;
	float* d_curPosCameraMapCenterImu;
	int* d_segments;

	cudaAllocateAndCopyToDevice((void**)&d_invCameraMatrix,
								invCameraMatrix,
								3*3*sizeof(float));
	//cudaAllocateAndCopyToDevice((void**)&d_distCoeffs,
	//							distCoeffs,
	//							5*sizeof(float));
	cudaAllocateAndCopyToDevice((void**)&d_curPosCameraMapCenterGlobal,
								curPosCameraMapCenterGlobal,
								4*4*sizeof(float));
	cudaAllocateAndCopyToDevice((void**)&d_curPosCameraMapCenterImu,
								curPosCameraMapCenterImu,
								4*4*sizeof(float));
	cudaAllocateAndCopyToDevice((void**)&d_segments,
								segments,
								numRows*numCols*sizeof(int));

	dim3 blockSize(32, 16, 1);
	dim3 gridSize((numCols + blockSize.x - 1) / blockSize.x,
					(numRows + blockSize.y - 1) / blockSize.y);
	compPointReprojection<<<gridSize, blockSize>>>(d_invCameraMatrix,
													d_distCoeffs,
													d_curPosCameraMapCenterGlobal,
													d_curPosCameraMapCenterImu,
													numRows,
													numCols,
													d_segments,
													mapSize,
													rasterSize);

	cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());

	cudaFree(d_invCameraMatrix);
	cudaFree(d_distCoeffs);
	cudaFree(d_curPosCameraMapCenterGlobal);
	cudaFree(d_curPosCameraMapCenterImu);
	cudaCopyFromDeviceAndFree(segments,
								d_segments,
								numRows*numCols*sizeof(int));

}


