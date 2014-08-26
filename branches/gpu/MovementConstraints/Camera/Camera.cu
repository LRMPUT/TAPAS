/*
 * CameraUtils.cu
 *
 *  Created on: Jul 30, 2014
 *      Author: robots
 */
//#include <opencv2/opencv.hpp>

#include <cuda_runtime.h>

#include <cstdio>
#include <iostream>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/scan.h>
#include <thrust/transform.h>
#include <thrust/sequence.h>
#include <thrust/copy.h>
#include <thrust/fill.h>
#include <thrust/replace.h>
#include <thrust/functional.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/iterator/transform_iterator.h>
#include <thrust/binary_search.h>
#include <thrust/adjacent_difference.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/iterator/counting_iterator.h>


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

typedef thrust::device_vector<unsigned char>::iterator ucharIter;
typedef thrust::tuple<ucharIter, ucharIter> uchar2IterTuple;
typedef thrust::zip_iterator<uchar2IterTuple> uchar2Tuple;

typedef thrust::device_vector<int>::iterator intIter;
typedef thrust::tuple<intIter, ucharIter> intUcharIterTuple;
typedef thrust::tuple<intIter, ucharIter, ucharIter> intUcharUcharIterTuple;
typedef thrust::zip_iterator<intUcharIterTuple> intUcharTuple;
typedef thrust::zip_iterator<intUcharUcharIterTuple> intUcharUcharTuple;

struct intUcharTupleMin{
  __host__ __device__
  const intUcharIterTuple& operator()(const intUcharIterTuple& lhs,
                  const intUcharIterTuple& rhs)
  {
	  if(thrust::get<0>(lhs) == thrust::get<0>(rhs)){
	  	if(thrust::get<1>(lhs) < thrust::get<1>(rhs)){
	    	return lhs;
	    }
	    else{
	    	return rhs;
	    }
	  }
	  else if(thrust::get<0>(lhs) < thrust::get<0>(rhs)){
		  return lhs;
	  }
	  else{
		  return rhs;
	  }
  }
};

//https://code.google.com/p/thrust/source/browse/examples/histogram.cu

void calcDenseHistograms(const int* const d_segments,
						const unsigned char* const d_bins,
						const unsigned int* d_segBeg,
						float* const d_histogram,
						int numBins,
						int numEntries)
{

  // find the end of each bin of values

  thrust::counting_iterator<int> search_begin(0);

  thrust::upper_bound(data.begin(), data.end(),
                      search_begin, search_begin + num_bins,
                      histogram.begin());

  // print the cumulative histogram
  print_vector("cumulative histogram", histogram);

  // compute the histogram by taking differences of the cumulative histogram
  thrust::adjacent_difference(histogram.begin(), histogram.end(),
                              histogram.begin());


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
	unsigned char *d_h, *d_s, *d_v, *d_hsBin, d_vBin;
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
	checkCudaErrors(cudaMalloc((void**)&d_hsBin, numRows*numCols*sizeof(unsigned char)));
	checkCudaErrors(cudaMalloc((void**)&d_vBin, numRows*numCols*sizeof(unsigned char)));

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
	thrust::device_ptr<int> d_segmentsImPtr(d_segmentsIm);
	thrust::device_vector<int> d_segmentsImHS(d_segmentsImPtr, d_segmentsImPtr + numRows*numCols);
	thrust::device_vector<int> d_segmentsImV(d_segmentsImPtr, d_segmentsImPtr + numRows*numCols);
	thrust::device_ptr<unsigned char> d_hPtr(d_h);
	thrust::device_ptr<unsigned char> d_sPtr(d_s);
	thrust::device_ptr<unsigned char> d_vPtr(d_v);

	thrust::device_ptr<unsigned char> d_hsBinPtr(d_hsBin);
	thrust::device_ptr<unsigned char> d_vBinPtr(d_vBin);

	intUcharTuple d_segHSBegin = thrust::make_zip_iterator(thrust::make_tuple(d_segmentsImHS, d_hsBinPtr));
	intUcharTuple d_segHSEnd = thrust::make_zip_iterator(thrust::make_tuple(d_segmentsImHS + numRows*numCols,
																			d_hsBinPtr + numRows*numCols));

	intUcharTuple d_segVBegin = thrust::make_zip_iterator(thrust::make_tuple(d_segmentsImV, d_vBinPtr));
	intUcharTuple d_segVEnd = thrust::make_zip_iterator(thrust::make_tuple(d_segmentsImV + numRows*numCols, d_vBinPtr + numRows*numCols));

	printf("compImageHistBinsHSV\n");
	compImageHistBinsHSV<<<gridSizeIm, blockSizeIm>>>(d_h,
												d_s,
												d_v,
												d_hsBins,
												d_vBins,
												numRows,
												numCols,
												d_featParams);

	//thrust::sort_by_key(d_segmentsImPtr, d_segmentsImPtr + numRows*numCols, d_imBegin);
	thrust::sort_by_key(d_segHSBegin, d_segHSEnd, d_hsBinPtr, d_hsBinPtr + numRows*numCols);
	thrust::sort_by_key(d_segVBegin, d_segVEnd, d_vBinPtr, d_vBinPtr + numRows*numCols);

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

	}



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
