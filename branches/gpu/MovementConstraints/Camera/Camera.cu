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
#include "cuPrintf.cu"

void cudaAllocateAndCopyToDevice(void** d_dst, const void* src, int size){
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

	checkCudaErrors(cudaFree(d_invCameraMatrix));
	checkCudaErrors(cudaFree(d_distCoeffs));
	checkCudaErrors(cudaFree(d_curPosCameraMapCenterGlobal));
	checkCudaErrors(cudaFree(d_curPosCameraMapCenterImu));
	cudaCopyFromDeviceAndFree(segments,
								d_segments,
								numRows*numCols*sizeof(int));

}

extern "C" void extractEntries(const unsigned char* const imageH,
								const unsigned char* const imageS,
								const unsigned char* const imageV,
								const float* const terrain,
								const int* const regionsOnImage,
								float* const feat,
								unsigned int* countPixelsEntries,
								unsigned int* countPointsEntries,
								const float* const cameraMatrix,
								const float* const distCoeffs,
								int numRows,
								int numCols,
								int numPoints,
								int numEntries,
								int descLen,
								const FeatParams* const featParams)
{
	unsigned char *d_h, *d_s, *d_v;
	float *d_terrain, *d_feat, *d_cameraMatrix, *d_distCoeffs;
	int *d_segmentsIm, *d_segmentsPoints;
	int* d_featInt;
	unsigned int *d_countSegmentsIm, *d_countSegmentsPoints;
	FeatParams *d_featParams;

	printf("cudaPrintfInit\n");
	cudaPrintfInit(10*1024*1024);
	printf("End cudaPrintfInit\n");

	cudaAllocateAndCopyToDevice((void**)&d_featParams,
									featParams,
									sizeof(FeatParams));
	cudaAllocateAndCopyToDevice((void**)&d_cameraMatrix,
								cameraMatrix,
								3*3*sizeof(float));
	//cudaAllocateAndCopyToDevice((void**)&d_distCoeffs,
	//							distCoeffs,
	//							5*sizeof(float));
	cudaAllocateAndCopyToDevice((void**)&d_h,
								imageH,
								numRows*numCols*sizeof(unsigned char));
	cudaAllocateAndCopyToDevice((void**)&d_s,
									imageS,
									numRows*numCols*sizeof(unsigned char));
	cudaAllocateAndCopyToDevice((void**)&d_v,
									imageV,
									numRows*numCols*sizeof(unsigned char));
	cudaAllocateAndCopyToDevice((void**)&d_terrain,
									(void*)terrain,
									numPoints*sizeof(float));
	cudaAllocateAndCopyToDevice((void**)&d_segmentsIm,
									regionsOnImage,
									numRows*numCols*sizeof(int));
	checkCudaErrors(cudaMalloc((void**)&d_segmentsPoints, numPoints*sizeof(int)));

	checkCudaErrors(cudaMalloc((void**)&d_feat, numEntries*descLen*sizeof(float)));
	checkCudaErrors(cudaMemset(d_feat, 0, numEntries*descLen*sizeof(float)));

	checkCudaErrors(cudaMalloc((void**)&d_featInt, numEntries*descLen*sizeof(int)));
	checkCudaErrors(cudaMemset(d_featInt, 0, numEntries*descLen*sizeof(int)));

	checkCudaErrors(cudaMalloc((void**)&d_countSegmentsIm, numEntries*sizeof(unsigned int)));
	checkCudaErrors(cudaMemset(d_countSegmentsIm, 0, numEntries*sizeof(unsigned int)));

	checkCudaErrors(cudaMalloc((void**)&d_countSegmentsPoints, numEntries*sizeof(unsigned int)));
	checkCudaErrors(cudaMemset(d_countSegmentsPoints, 0, numEntries*sizeof(unsigned int)));

	dim3 blockSizeIm(32, 16, 1);
	dim3 gridSizeIm((numCols + blockSizeIm.x - 1) / blockSizeIm.x,
					(numRows + blockSizeIm.y - 1) / blockSizeIm.y);
	//printf("gridSizeIm = (%d, %d, %d)\n", gridSizeIm.x, gridSizeIm.y, gridSizeIm.z);

	dim3 blockSizePoints(512, 1, 1);
	dim3 gridSizePoints((numPoints + blockSizePoints.x - 1) / blockSizePoints.x, 1, 1);

	dim3 blockSizeEntries(512, 1, 1);
	dim3 gridSizeEntries((numEntries + blockSizeEntries.x - 1) / blockSizeEntries.x, 1, 1);
	//printf("gridSizePoints = (%d, %d, %d)\n", gridSizePoints.x, gridSizePoints.y, gridSizePoints.z);

	//printf("d_segmentsIm = %p, d_countSegmentsIm = %p, numRows = %d, numCols = %d, numEntries = %d\n", d_segmentsIm, d_countSegmentsIm, numRows, numCols, numEntries);
	//printf("d_terrain = %p, d_segmentsPoints = %p\n", d_terrain, d_segmentsPoints);
	//precomputing
	printf("countSegmentPixels\n");
	countSegmentPixels<<<gridSizeIm, blockSizeIm>>>(d_segmentsIm,
													d_countSegmentsIm,
													numRows,
													numCols,
													numEntries);
	cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());

	if(numPoints > 0){
		printf("compPointProjection\n");
		compPointProjection<<<gridSizePoints, blockSizePoints>>>(d_terrain,
																d_segmentsIm,
																d_segmentsPoints,
																d_cameraMatrix,
																d_distCoeffs,
																numPoints,
																numRows,
																numCols);
		cudaDeviceSynchronize();
		checkCudaErrors(cudaGetLastError());

		printf("countSegmentPoints\n");
		countSegmentPoints<<<gridSizePoints, blockSizePoints>>>(d_segmentsPoints,
																d_countSegmentsPoints,
																numPoints);
		cudaDeviceSynchronize();
		checkCudaErrors(cudaGetLastError());
	}

	//extracting features
	int startRow = 0;

	printf("compImageHistHSV\n");
	compImageHistHSV<<<gridSizeIm, blockSizeIm>>>(d_h,
												d_s,
												d_v,
												d_countSegmentsIm,
												d_featInt + startRow*numEntries,
												d_segmentsIm,
												numRows,
												numCols,
												numEntries,
												d_featParams);

	scaleData<<<gridSizeEntries, blockSizeEntries>>>(d_featInt + startRow*numEntries,
													d_feat + startRow*numEntries,
													d_countSegmentsIm,
													featParams->histHLen * featParams->histSLen + featParams->histVLen,
													numEntries);
	cudaPrintfDisplay(stdout, true);
	printf("End displaying cuPrintf\n");
	cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());
	startRow += featParams->histHLen * featParams->histSLen + featParams->histVLen;

	printf("compImageMeanHSV\n");
	compImageMeanHSV<<<gridSizeIm, blockSizeIm>>>(d_h,
												d_s,
												d_v,
												d_countSegmentsIm,
												d_featInt + startRow*numEntries,
												d_segmentsIm,
												numRows,
												numCols,
												numEntries,
												d_featParams);
	cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());
	int meanHSVStartRow = startRow;
	startRow += 3;

	printf("compImageCovarHSV\n");
	compImageCovarHSV<<<gridSizeIm, blockSizeIm>>>(d_h,
													d_s,
													d_v,
													d_countSegmentsIm,
													d_feat + meanHSVStartRow*numEntries,
													d_featInt + startRow*numEntries,
													d_segmentsIm,
													numRows,
													numCols,
													numEntries,
													d_featParams);
	cudaDeviceSynchronize();
	checkCudaErrors(cudaGetLastError());
	startRow += 9;

	if(numPoints > 0){
		printf("compTerrainHistDI");
		compTerrainHistDI<<<gridSizePoints, blockSizePoints>>>(d_terrain,
																d_segmentsPoints,
																d_countSegmentsPoints,
																d_feat + startRow*numEntries,
																numPoints,
																numEntries,
																d_featParams);
		cudaPrintfDisplay(stdout, true);
		printf("End displaying cuPrintf\n");
		cudaDeviceSynchronize();
		checkCudaErrors(cudaGetLastError());
	}
	startRow += featParams->histDLen * featParams->histILen;

	if(numPoints > 0){
		printf("compTerrainMeanDI\n");
		compTerrainMeanDI<<<gridSizePoints, blockSizePoints>>>(d_terrain,
																d_segmentsPoints,
																d_countSegmentsPoints,
																d_feat + startRow*numEntries,
																numPoints,
																numEntries,
																d_featParams);
		cudaDeviceSynchronize();
		checkCudaErrors(cudaGetLastError());
	}
	int meanDIStartRow = startRow;
	startRow += 2;

	if(numPoints > 0){
		printf("compTerrainCovarDI");
		compTerrainCovarDI<<<gridSizePoints, blockSizePoints>>>(d_terrain,
																d_segmentsPoints,
																d_countSegmentsPoints,
																d_feat + meanDIStartRow * numEntries,
																d_feat + startRow*numEntries,
																numPoints,
																numEntries,
																d_featParams);
		cudaDeviceSynchronize();
		checkCudaErrors(cudaGetLastError());
	}
	startRow += 4;

	cudaPrintfEnd();

	checkCudaErrors(cudaFree(d_featParams));
	checkCudaErrors(cudaFree(d_cameraMatrix));
	//checkCudaErrors(cudaFree(d_distCoeffs));
	checkCudaErrors(cudaFree(d_h));
	checkCudaErrors(cudaFree(d_s));
	checkCudaErrors(cudaFree(d_v));
	checkCudaErrors(cudaFree(d_terrain));
	checkCudaErrors(cudaFree(d_segmentsIm));
	checkCudaErrors(cudaFree(d_segmentsPoints));
	cudaCopyFromDeviceAndFree(feat, d_feat, numEntries*descLen*sizeof(float));
	cudaCopyFromDeviceAndFree(countPixelsEntries, d_countSegmentsIm, numEntries*sizeof(unsigned int));
	cudaCopyFromDeviceAndFree(countPointsEntries, d_countSegmentsPoints, numEntries*sizeof(unsigned int));
}
