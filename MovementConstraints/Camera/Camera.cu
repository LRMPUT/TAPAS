/*
	Copyright (c) 2014,	TAPAS Team:
	-Michal Nowicki (michal.nowicki@put.poznan.pl),
	-Jan Wietrzykowski (jan.wietrzykowski@cie.put.poznan.pl).
	Poznan University of Technology
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice,
	this list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice,
	this list of conditions and the following disclaimer in the documentation
	and/or other materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
	THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
	ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
	AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
	OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <cuda_runtime.h>

#include <cstdio>
#include <iostream>
#include <iomanip>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/scan.h>
#include <thrust/transform.h>
#include <thrust/sequence.h>
#include <thrust/copy.h>
#include <thrust/fill.h>
#include <thrust/replace.h>
#include <thrust/unique.h>
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
typedef thrust::tuple<unsigned char, unsigned char, unsigned char> uchar3Tuple;
typedef thrust::tuple<ucharIter, ucharIter> uchar2IterTuple;
typedef thrust::zip_iterator<uchar2IterTuple> uchar2Zip;
typedef thrust::tuple<ucharIter, ucharIter, ucharIter> uchar3IterTuple;
typedef thrust::zip_iterator<uchar3IterTuple> uchar3Zip;

typedef thrust::device_vector<int>::iterator intIter;
typedef thrust::tuple<int, int> int2Tuple;
typedef thrust::tuple<intIter, intIter> int2IterTuple;
typedef thrust::zip_iterator<int2IterTuple> int2Zip;

struct int2TupleMin{
  __host__ __device__
  bool operator()(const int2Tuple& lhs,
                  const int2Tuple& rhs)
  {
	  if(thrust::get<0>(lhs) == thrust::get<0>(rhs)){
	  	if(thrust::get<1>(lhs) < thrust::get<1>(rhs)){
	    	return true;
	    }
	    else{
	    	return false;
	    }
	  }
	  else if(thrust::get<0>(lhs) < thrust::get<0>(rhs)){
		  return true;
	  }
	  else{
		  return false;
	  }
  }
};

//https://code.google.com/p/thrust/source/browse/examples/histogram.cu

template <typename T, class InputIterator>
void printVector(const std::string& name, InputIterator vBeg, InputIterator vEnd)
{
	std::cout << " " << std::setw(20) << name << " ";
	thrust::copy(vBeg, vEnd, std::ostream_iterator<T>(std::cout, " "));
	std::cout << std::endl;
}

//https://code.google.com/p/thrust/source/browse/examples/summed_area_table.cu

// convert a linear index to a linear index in the transpose
struct transpose_index : public thrust::unary_function<size_t,size_t>
{
    size_t n, m;

    __host__ __device__
    transpose_index(size_t _n, size_t _m) :  n(_n), m(_m) {}

    __host__ __device__
    size_t operator()(size_t linear_index)
    {
        size_t i = linear_index / n;
        size_t j = linear_index % n;

        return m * j + i;
    }
};

struct modulusUnary : public thrust::unary_function<int, int>
{
    int mod;

    __host__ __device__
    modulusUnary(int imod) :  mod(imod) {}

    __host__ __device__
    int operator()(int x)
    {
        return x % mod;
    }
};

// transpose an M-by-N array
template <typename InputIterator, typename OutputIterator>
void transpose(size_t m, size_t n, InputIterator src, OutputIterator dst)
{
    thrust::counting_iterator<size_t> indices(0);

    //printVector<size_t>("gather inices",
    //					thrust::make_transform_iterator(indices, transpose_index(n, m)),
    //					thrust::make_transform_iterator(indices, transpose_index(n, m)) + n*m);
    thrust::scatter(src, src + n*m,
                   thrust::make_transform_iterator(indices, transpose_index(n, m)),
                   dst);
}

//https://code.google.com/p/thrust/source/browse/examples/histogram.cu

void calcDenseHists(int2Zip d_vals,
						const unsigned int* const d_segmentsEnds,
						float* const d_histogram,
						int numVals,
						int numBins,
						int numEntries)
{

	// find the end of each bin of values

	thrust::constant_iterator<int> numBinsIter(numBins);
	thrust::counting_iterator<int> sequenceIter(0);

	thrust::device_vector<int> d_entriesVal(numBins*numEntries);
	thrust::device_vector<int> d_binsVal(numBins*numEntries);
	thrust::device_vector<int> d_segmentsCount(numBins*numEntries);
	thrust::device_ptr<float> d_histPtr(d_histogram);
	thrust::device_ptr<const unsigned int> d_segmentsEndsPtr(d_segmentsEnds);

	thrust::transform(sequenceIter, sequenceIter + numBins*numEntries, numBinsIter, d_entriesVal.begin(), thrust::divides<int>());
	thrust::transform(sequenceIter, sequenceIter + numBins*numEntries, numBinsIter, d_binsVal.begin(), thrust::modulus<int>());

	int2Zip d_searchValsBeg = thrust::make_zip_iterator(thrust::make_tuple(d_entriesVal.begin(), d_binsVal.begin()));
	int2Zip d_searchValsEnd = thrust::make_zip_iterator(thrust::make_tuple(d_entriesVal.end(), d_binsVal.end()));
	thrust::upper_bound(d_vals, d_vals + numVals,
					  d_searchValsBeg, d_searchValsEnd,
					  d_histPtr);

	// print the cumulative histogram
	//printVector<int>("d_searchVals entries", d_entriesVal.begin(), d_entriesVal.end());
	//printVector<int>("d_searchVals bins", d_binsVal.begin(), d_binsVal.end());
	//printVector<float>("cumulative histogram", d_histPtr, d_histPtr + numBins*numEntries);

	// compute the histogram by taking differences of the cumulative histogram
	thrust::adjacent_difference(d_histPtr, d_histPtr + numBins*numEntries,
								d_histPtr);


	//printVector<float>("histogram", d_histPtr, d_histPtr + numBins*numEntries);
	//transpose
	transpose(numEntries, numBins, d_histPtr, d_histPtr);

	//printVector<float>("transposed histogram", d_histTmp.begin(), d_histTmp.end());
	//printVector<float>("transposed histogram", d_histPtr, d_histPtr + numBins*numEntries);

	thrust::adjacent_difference(d_segmentsEndsPtr, d_segmentsEndsPtr + numEntries,
								d_segmentsCount.begin());

	typedef thrust::device_vector<int>::iterator IntIter;
	typedef thrust::transform_iterator<modulusUnary, thrust::counting_iterator<int> > TransIter;

	TransIter indicesIter(sequenceIter, modulusUnary(numEntries));
	thrust::permutation_iterator<IntIter, TransIter> permIterSegCount(d_segmentsCount.begin(), indicesIter);

	//printVector<int>("d_segmentsEnd", d_segmentsEndsPtr, d_segmentsEndsPtr + numEntries);
	//printVector<int>("permIterSegCount", permIterSegCount, permIterSegCount + numBins*numEntries);
	thrust::transform(d_histPtr, d_histPtr + numBins*numEntries,
						permIterSegCount,
						d_histPtr,
						thrust::divides<float>());

	//printVector<float>("normalized histogram", d_histPtr, d_histPtr + numBins*numEntries);
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
								int descLen,
								const FeatParams* const featParams)
{
	unsigned char *d_h, *d_s, *d_v;
	float *d_terrain, *d_feat, *d_cameraMatrix, *d_distCoeffs;
	int *d_segmentsIm, *d_segmentsImUniq, *d_segmentsPoints, *d_hsBin, *d_vBin;
	//int* d_featInt;
	unsigned int *d_segmentsImEnds, *d_segmentsPointsEnds;
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
	checkCudaErrors(cudaMalloc((void**)&d_segmentsImUniq, numRows*numCols*sizeof(int)));
	checkCudaErrors(cudaMalloc((void**)&d_hsBin, numRows*numCols*sizeof(int)));
	checkCudaErrors(cudaMalloc((void**)&d_vBin, numRows*numCols*sizeof(int)));

	checkCudaErrors(cudaMalloc((void**)&d_segmentsPoints, numPoints*sizeof(int)));

	dim3 blockSizeIm(32, 16, 1);
	dim3 gridSizeIm((numCols + blockSizeIm.x - 1) / blockSizeIm.x,
					(numRows + blockSizeIm.y - 1) / blockSizeIm.y);
	//printf("gridSizeIm = (%d, %d, %d)\n", gridSizeIm.x, gridSizeIm.y, gridSizeIm.z);

	dim3 blockSizePoints(512, 1, 1);
	dim3 gridSizePoints((numPoints + blockSizePoints.x - 1) / blockSizePoints.x, 1, 1);

	//dim3 blockSizeEntries(512, 1, 1);
	//dim3 gridSizeEntries((numEntries + blockSizeEntries.x - 1) / blockSizeEntries.x, 1, 1);
	//printf("gridSizePoints = (%d, %d, %d)\n", gridSizePoints.x, gridSizePoints.y, gridSizePoints.z);

	//printf("d_segmentsIm = %p, d_countSegmentsIm = %p, numRows = %d, numCols = %d, numEntries = %d\n", d_segmentsIm, d_countSegmentsIm, numRows, numCols, numEntries);
	//printf("d_terrain = %p, d_segmentsPoints = %p\n", d_terrain, d_segmentsPoints);

	//precomputing
	thrust::device_ptr<int> d_segmentsImPtr(d_segmentsIm);
	thrust::device_ptr<int> d_segmentsImUniqPtr(d_segmentsImUniq);
	thrust::device_ptr<unsigned char> d_hPtr(d_h);
	thrust::device_ptr<unsigned char> d_sPtr(d_s);
	thrust::device_ptr<unsigned char> d_vPtr(d_v);

	thrust::device_ptr<int> d_hsBinPtr(d_hsBin);
	thrust::device_ptr<int> d_vBinPtr(d_vBin);

	//sort
	printf("Sorting\n");
	uchar3Zip d_hsvIterBeg = thrust::make_zip_iterator(thrust::make_tuple(d_hPtr, d_sPtr, d_vPtr));
	uchar3Zip d_hsvIterEnd = thrust::make_zip_iterator(thrust::make_tuple(d_hPtr + numRows*numCols, d_sPtr + numRows*numCols, d_vPtr + numRows*numCols));

	for(int i = 0; i < 100; i++){
		uchar3Tuple tmp;
		uchar3Zip iter = d_hsvIterBeg + i;
		thrust::copy(iter, iter + 1, &tmp);
		thrust::device_reference<unsigned char> val0 = thrust::get<0>(*(d_hsvIterBeg + i));
		thrust::device_reference<unsigned char> val1 = thrust::get<1>(*(d_hsvIterBeg + i));
		std::cout << "(" << (int)val0 << ", " << (int)val1 << ") ";
		//std::cout << "(" << tmp << ") ";
	}
	std::cout << std::endl;

	thrust::sort_by_key(d_segmentsImPtr, d_segmentsImPtr + numRows*numCols, d_hsvIterBeg);

	//unique
	printf("Unique\n");
	//printVector("d_segmentsIm", d_segmentsImPtr, d_segmentsImPtr + 6000);
	thrust::device_ptr<int> d_segmentsImUniqPtrEnd = thrust::unique_copy(d_segmentsImPtr,
																	d_segmentsImPtr + numRows*numCols,
																	d_segmentsImUniqPtr);
	printf("end unique_copy()\n");
	int numEntries = d_segmentsImUniqPtrEnd - d_segmentsImUniqPtr;
	printf("numEntries = %d\n", numEntries);
	//int numEntries = 87;
	//printVector<int>("d_segmentsImUniq", d_segmentsImUniqPtr,
	//								d_segmentsImUniqPtr + numEntries);

	checkCudaErrors(cudaMalloc((void**)&d_feat, numEntries*descLen*sizeof(float)));
	checkCudaErrors(cudaMemset(d_feat, 0, numEntries*descLen*sizeof(float)));

	//checkCudaErrors(cudaMalloc((void**)&d_featInt, numEntries*descLen*sizeof(int)));
	//checkCudaErrors(cudaMemset(d_featInt, 0, numEntries*descLen*sizeof(int)));

	checkCudaErrors(cudaMalloc((void**)&d_segmentsImEnds, numEntries*sizeof(unsigned int)));
	checkCudaErrors(cudaMemset(d_segmentsImEnds, 0, numEntries*sizeof(unsigned int)));

	checkCudaErrors(cudaMalloc((void**)&d_segmentsPointsEnds, numEntries*sizeof(unsigned int)));
	checkCudaErrors(cudaMemset(d_segmentsPointsEnds, 0, numEntries*sizeof(unsigned int)));

	thrust::device_ptr<unsigned int> d_segmentsImEndsPtr(d_segmentsImEnds);

	printf("upper_bound\n");
	thrust::upper_bound(d_segmentsImPtr, d_segmentsImPtr + numRows*numCols,
						d_segmentsImUniqPtr, d_segmentsImUniqPtr + numEntries,
						d_segmentsImEndsPtr);
	//printVector<unsigned int>("d_segmentsImEnds", d_segmentsImEndsPtr, d_segmentsImEndsPtr + numEntries);
	printf("lower_bound\n");
	thrust::lower_bound(d_segmentsImUniqPtr, d_segmentsImUniqPtr + numEntries,
						d_segmentsImPtr, d_segmentsImPtr + numRows*numCols,
						d_segmentsImPtr);

	//printVector("d_segmentsIm", d_segmentsImPtr, d_segmentsImPtr + 6000);

	int startRow = 0;

	printf("compImageHistBinsHSV\n");
	compImageHistBinsHSV<<<gridSizeIm, blockSizeIm>>>(d_h,
												d_s,
												d_v,
												d_hsBin,
												d_vBin,
												numRows,
												numCols,
												d_featParams);


	thrust::device_vector<int> d_segmentsImHS(d_segmentsImPtr, d_segmentsImPtr + numRows*numCols);
	thrust::device_vector<int> d_segmentsImV(d_segmentsImPtr, d_segmentsImPtr + numRows*numCols);

	int2Zip d_segHSBegin = thrust::make_zip_iterator(thrust::make_tuple(d_segmentsImHS.begin(), d_hsBinPtr));
	int2Zip d_segHSEnd = thrust::make_zip_iterator(thrust::make_tuple(d_segmentsImHS.begin() + numRows*numCols,
																			d_hsBinPtr + numRows*numCols));

	int2Zip d_segVBegin = thrust::make_zip_iterator(thrust::make_tuple(d_segmentsImV.begin(), d_vBinPtr));
	int2Zip d_segVEnd = thrust::make_zip_iterator(thrust::make_tuple(d_segmentsImV.begin() + numRows*numCols,
																		d_vBinPtr + numRows*numCols));

	//printVector<int>("d_segmentsImHS", d_segmentsImHS.begin(), d_segmentsImHS.end());
	//printVector<int>("d_hsBin", d_hsBinPtr, d_hsBinPtr + numRows*numCols);
	//printVector<int>("d_h", d_hPtr, d_hPtr + 100);
	//printVector<int>("d_s", d_sPtr, d_sPtr + 100);
	//thrust::sort_by_key(d_segmentsImPtr, d_segmentsImPtr + numRows*numCols, d_imBegin);
	/*for(int i = 0; i < 100; i++){
		//int2Tuple tmp;
		//int2Zip iter = d_segHSBegin + i;
		//thrust::copy(iter, iter + 1, &tmp);
		thrust::device_reference<int> val0 = thrust::get<0>(*(d_segHSBegin + i));
		thrust::device_reference<int> val1 = thrust::get<1>(*(d_segHSBegin + i));
		std::cout << i << ":(" << val0 << ", " << val1 << ") ";
		//std::cout << "(" << tmp << ") ";
	}
	std::cout << std::endl;*/

	printf("sort HS\n");
	thrust::sort(d_segHSBegin, d_segHSEnd);
	printf("sort V\n");
	thrust::sort(d_segVBegin, d_segVEnd);

	printf("calcDenseHists\n");
	calcDenseHists(d_segHSBegin,
						d_segmentsImEnds,
						d_feat + startRow,
						numRows*numCols,
						featParams->histHLen * featParams->histSLen,
						numEntries);

	startRow += featParams->histHLen * featParams->histSLen;

	printf("calcDenseHists\n");
	calcDenseHists(d_segVBegin,
						d_segmentsImEnds,
						d_feat + startRow,
						numRows*numCols,
						featParams->histVLen,
						numEntries);

	startRow += featParams->histVLen;

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
	cudaCopyFromDeviceAndFree(countPixelsEntries, d_segmentsImEnds, numEntries*sizeof(unsigned int));
	cudaCopyFromDeviceAndFree(countPointsEntries, d_segmentsPointsEnds, numEntries*sizeof(unsigned int));
}
