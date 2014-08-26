
#ifndef CAMERA_KERNELS_CU
#define CAMERA_KERNELS_CU

#include "CameraCuda.h"
#include "cuPrintf.cu"

__device__ __forceinline__ void multMat(const float* const d_A,
										const float* const d_B,
										float* const d_C,
										int nRowsA,
										int nColsA,
										int nRowsB,
										int nColsB)
{
	int nRowsC = nRowsA;
	int nColsC = nColsB;
	for(int row = 0; row < nRowsA; row++){
		for(int col = 0; col < nColsB; col++){
			d_C[row * nColsC + col] = 0;
			for(int idx = 0; idx < nColsA; idx++){
				d_C[row * nColsC + col] += d_A[row * nColsA + idx]*d_B[idx * nColsB + col];
			}
		}
	}
}

__device__ inline float atomicAddFloatNew(float* address, float value){

	float old = value;
	float ret=atomicExch(address, 0.0f);
	float new_old=ret+old;

	while ((old = atomicExch(address, new_old))!=0.0f){
		new_old = atomicExch(address, 0.0f);
		new_old += old;
	}

	return ret;
}

__device__ __forceinline__ float atomicAddFloat(float* address, float val)
{

	/*if(val > 0.0f){
		float old = val;
		float ret=atomicExch(address, 0.0f);
		float new_old=ret+old;

		int counter = 0;
		while((old = atomicExch(address, new_old)) != 0.0f){
			new_old = atomicExch(address, 0.0f);
			new_old += old;
			counter++;
		}
		cuPrintf("counter = %d\n", counter);
		return ret;
	}
	else{
		return *address;
	}*/

    unsigned int* address_as_uint = (unsigned int*)address;
    unsigned int old = *address_as_uint, assumed;
    int counter = 0;
    do {
        assumed = old;
        old = atomicCAS(address_as_uint, assumed,
                        __float_as_int(val +
                               __int_as_float(assumed)));

		counter++;
    } while (assumed != old);
	cuPrintf("counter = %d\n", counter);
    return __int_as_float(old);
}


__global__ void compPointReprojection(float* d_invCameraMatrix,
										float* d_distCoeffs,
										float* d_curPosCameraMapCenterGlobal,
										float* d_curPosCameraMapCenterImu,
										int numRows,
										int numCols,
										int* d_segments,
										int mapSize,
										int rasterSize)
{
	//TODO Add distortion coefficients
	int idxX = (blockIdx.x * blockDim.x) + threadIdx.x;
	int idxY = (blockIdx.y * blockDim.y) + threadIdx.y;

	if(idxX < numCols && idxY < numRows){
		float pointIm[3] = {idxX,
							idxY,
							1};
		float pointCamNN[3];
		multMat(d_invCameraMatrix, pointIm, pointCamNN, 3, 3, 3, 1);

		float t31 = d_curPosCameraMapCenterGlobal[2*4 + 0];
		float t32 = d_curPosCameraMapCenterGlobal[2*4 + 1];
		float t33 = d_curPosCameraMapCenterGlobal[2*4 + 2];
		float t34 = d_curPosCameraMapCenterGlobal[2*4 + 3];
		float s = (-t34) / (t31 * pointCamNN[0] + t32 * pointCamNN[1] + t33 * pointCamNN[2]); //at z_glob = 0

		float pointCam[4] = {pointCamNN[0]*s,
							pointCamNN[1]*s,
							pointCamNN[2]*s,
							1};
		float pointMapCenter[4];
		multMat(d_curPosCameraMapCenterImu, pointCam, pointMapCenter, 4, 4, 4, 1);

		int xSegm = pointMapCenter[0]/rasterSize + mapSize/2;
		int ySegm = pointMapCenter[1]/rasterSize + mapSize/2;
		//cout << r << ":" << c << " = (" << xSegm << ", " << ySegm << ")" << endl;
		d_segments[idxY * numCols + idxX] = xSegm*mapSize + ySegm;
	}
}

__global__ void countSegmentPixels(const int* const d_segments,
									unsigned int* const d_countSegments,
									int numRows,
									int numCols,
									int numEntries)
{
	int idxX = (blockIdx.x * blockDim.x) + threadIdx.x;
	int idxY = (blockIdx.y * blockDim.y) + threadIdx.y;
	int idx1d = idxY * numCols + idxX;

	//cuPrintf("idxX = %d, idxY = %d, idx1d = %d\n", idxX, idxY, idx1d);
	if(idxX < numCols && idxY < numRows){
		int entry = d_segments[idx1d];
		//cuPrintf("entry = %d\n", entry);
		if(entry >= 0){
			atomicInc(&d_countSegments[entry], 0xffffffff);
		}
	}
}

__global__ void compPointProjection(const float* const d_terrain,
								const int* const d_imageSeg,
								int* const d_pointSeg,
								const float* const d_cameraMatrix,
								const float* const d_distCoeffs,
								int numPoints,
								int numRows,
								int numCols)
{
	//TODO Add distortion coefficients
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if(idx < numPoints){
		float point3d[3] = {d_terrain[0 * numPoints + idx],
							d_terrain[1 * numPoints + idx],
							d_terrain[2 * numPoints + idx]};
		float point2d[3];
		multMat(d_cameraMatrix, point3d, point2d, 3, 3, 3, 1);
		float s = point2d[2];
		point2d[0] /= s;
		point2d[1] /= s;
		point2d[2] /= s;
		int pX = (int)point2d[0];
		int pY = (int)point2d[1];
		if(pX >= 0 && pX < numCols && pY >= 0 && pY < numRows){
			d_pointSeg[idx] = d_imageSeg[pY * numCols + pX];
		}
		else{
			d_pointSeg[idx] = -1;
		}
	}
}

__global__ void countSegmentPoints(const int* const d_segments,
								unsigned int* const d_countSegments,
								int numPoints)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if(idx < numPoints){
		int entry = d_segments[idx];
		if(entry >= 0){
			atomicInc(&d_countSegments[entry], 0xffffffff);
		}
	}
}


__global__ void scaleData(const int* const d_hist,
							float* const d_feat,
							const unsigned int* d_countSegments,
							int numVals,
							int numEntries)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if(idx < numEntries){
		for(int v = 0; v < numVals; v++){
			d_feat[v * numEntries + idx] = (float)d_hist[v * numEntries + idx] / d_countSegments[idx];
		}
	}
}

__global__ void compImageHistHSV(const unsigned char* const d_h,
								const unsigned char* const d_s,
								const unsigned char* const d_v,
								const unsigned int* const d_countSegments,
								int* const d_hist,
								const int* const d_segments,
								int numRows,
								int numCols,
								int numEntries,
								const FeatParams* const d_featParams)
{
	int idxX = (blockIdx.x * blockDim.x) + threadIdx.x;
	int idxY = (blockIdx.y * blockDim.y) + threadIdx.y;
	int idx1d = idxY * numCols + idxX;

	if(idxX < numCols && idxY < numRows){
		int entry = d_segments[idx1d];
		if(entry >= 0){
			//int entryCount = d_countSegments[entry];

			int startRow = 0;

			//histHS
			int hVal = (d_h[idx1d] & 0xff);
			int sVal = (d_s[idx1d] & 0xff);
			int vVal = (d_v[idx1d] & 0xff);
			int hBin = (hVal - d_featParams->histHRangeMin) *
					 d_featParams->histHLen /(d_featParams->histHRangeMax - d_featParams->histHRangeMin);
			int sBin = (sVal - d_featParams->histSRangeMin) *
					 d_featParams->histSLen / (d_featParams->histSRangeMax - d_featParams->histSRangeMin);
			hBin = max(min(hBin, d_featParams->histHLen - 1), 0);
			sBin = max(min(sBin, d_featParams->histSLen - 1), 0);
			atomicInc((unsigned int*)&d_hist[(startRow + sBin * d_featParams->histHLen + hBin)* numEntries + entry], 0x8fffff);
			startRow += d_featParams->histHLen * d_featParams->histSLen;

			//histV
			int vBin  = (vVal - d_featParams->histVRangeMin) *
					d_featParams->histVLen / (d_featParams->histVRangeMax - d_featParams->histVRangeMin);
			vBin = max(min(vBin, d_featParams->histVLen - 1), 0);
			atomicInc((unsigned int*)&d_hist[(startRow + vBin) * numEntries + entry], 0x8fffff);
			startRow += d_featParams->histVLen;

			//cuPrintf("entry = %d, entryCount = %d, 1.0/entryCount = %f\n", entry, entryCount, (float)1.0/entryCount);
			//cuPrintf("hBin = %d, sBin = %d, vBin = %d, hVal = %d, sVal = %d, vVal = %d, hLen = %d, hMax = %f\n",
			//		hBin, sBin, vBin, hVal, sVal, vVal, d_featParams->histHLen, d_featParams->histHRangeMax);
		}
	}
}

__global__ void compImageMeanHSV(const unsigned char* const d_h,
								const unsigned char* const d_s,
								const unsigned char* const d_v,
								const unsigned int* const d_countSegments,
								int* const d_mean,
								const int* const d_segments,
								int numRows,
								int numCols,
								int numEntries,
								const FeatParams* const d_featParams)
{
	int idxX = (blockIdx.x * blockDim.x) + threadIdx.x;
	int idxY = (blockIdx.y * blockDim.y) + threadIdx.y;
	int idx1d = idxY * numCols + idxX;

	if(idxX < numCols && idxY < numRows){
		int entry = d_segments[idx1d];
		if(entry >= 0){
			//int entryCount = d_countSegments[entry];

			int startRow = 0;
			//meanHSV
			//atomicAdd(&d_mean[(startRow + 0) * numEntries + entry], (d_h[idx1d] & 0xff));
			//atomicAdd(&d_mean[(startRow + 1) * numEntries + entry], (d_s[idx1d] & 0xff));
			//atomicAdd(&d_mean[(startRow + 2) * numEntries + entry], (d_v[idx1d] & 0xff));
			startRow += 3;
		}
	}
}

__global__ void compImageCovarHSV(const unsigned char* const d_h,
								const unsigned char* const d_s,
								const unsigned char* const d_v,
								const unsigned int* const d_countSegments,
								const float* const d_mean,
								int* const d_covar,
								const int* const d_segments,
								int numRows,
								int numCols,
								int numEntries,
								const FeatParams* const d_featParams)
{
	int idxX = (blockIdx.x * blockDim.x) + threadIdx.x;
	int idxY = (blockIdx.y * blockDim.y) + threadIdx.y;
	int idx1d = idxY * numCols + idxX;

	if(idxX < numCols && idxY < numRows){
		int entry = d_segments[idx1d];
		if(entry >= 0){
			//int entryCount = d_countSegments[entry];

			int startRow = 0;
			//covar HSV
			int c11 = (d_h[idx1d] -  d_mean[0 * numEntries + entry])*(d_h[idx1d] -  d_mean[0 * numEntries + entry]);
			int c12 = (d_h[idx1d] -  d_mean[0 * numEntries + entry])*(d_s[idx1d] -  d_mean[1 * numEntries + entry]);
			int c13 = (d_h[idx1d] -  d_mean[0 * numEntries + entry])*(d_v[idx1d] -  d_mean[2 * numEntries + entry]);
			int c22 = (d_s[idx1d] -  d_mean[1 * numEntries + entry])*(d_s[idx1d] -  d_mean[1 * numEntries + entry]);
			int c23 = (d_s[idx1d] -  d_mean[1 * numEntries + entry])*(d_v[idx1d] -  d_mean[2 * numEntries + entry]);
			int c33 = (d_v[idx1d] -  d_mean[2 * numEntries + entry])*(d_v[idx1d] -  d_mean[2 * numEntries + entry]);
			/*atomicAdd(&d_covar[(startRow + 0) * numEntries + entry], c11);
			atomicAdd(&d_covar[(startRow + 1) * numEntries + entry], c12);
			atomicAdd(&d_covar[(startRow + 2) * numEntries + entry], c13);
			atomicAdd(&d_covar[(startRow + 3) * numEntries + entry], c12);
			atomicAdd(&d_covar[(startRow + 4) * numEntries + entry], c22);
			atomicAdd(&d_covar[(startRow + 5) * numEntries + entry], c23);
			atomicAdd(&d_covar[(startRow + 6) * numEntries + entry], c12);
			atomicAdd(&d_covar[(startRow + 7) * numEntries + entry], c23);
			atomicAdd(&d_covar[(startRow + 8) * numEntries + entry], c33);*/
			startRow += 9;
		}
	}
}

__global__ void compTerrainHistDI(const float* const d_terrain,
								const int* const d_segments,
								const unsigned int* const d_countSegments,
								float* const d_feat,
								int numPoints,
								int numEntries,
								const FeatParams* const d_featParams)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if(idx < numPoints){
		int entry = d_segments[idx];
		cuPrintf("idx = %d, entry = %d\n", idx, entry);
		/*if(entry >= 0){
			int entryCount = d_countSegments[entry];
			//distance - 4th row
			int dBin = (d_terrain[4 * numPoints + idx] - d_featParams->histDRangeMin) /
								(d_featParams->histDRangeMax - d_featParams->histDRangeMin) * d_featParams->histDLen;
			//intensity - 5th row
			int iBin = (d_terrain[5 * numPoints + idx] - d_featParams->histIRangeMin) /
											(d_featParams->histIRangeMax - d_featParams->histIRangeMin) * d_featParams->histILen;

			int startRow = 0;
			int bin1d = iBin*d_featParams->histDLen + dBin;
			atomicAddFloat(&d_feat[(startRow + bin1d) * numEntries + entry], (float)1.0/entryCount);
			startRow += d_featParams->histDLen * d_featParams->histILen;
		}*/
	}
}

__global__ void compTerrainMeanDI(const float* const d_terrain,
								const int* const d_segments,
								const unsigned int* const d_countSegments,
								float* const d_feat,
								int numPoints,
								int numEntries,
								const FeatParams* const d_featParams)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if(idx < numPoints){
		int entry = d_segments[idx];
		if(entry >= 0){
			int entryCount = d_countSegments[entry];

			int startRow = 0;
			//meanDI
			atomicAddFloat(&d_feat[(startRow + 0) * numEntries + entry], (float)d_terrain[4 * numPoints + idx]/entryCount);
			atomicAddFloat(&d_feat[(startRow + 1) * numEntries + entry], (float)d_terrain[5 * numPoints + idx]/entryCount);
			startRow += 2;
		}
	}
}

__global__ void compTerrainCovarDI(const float* const d_terrain,
								const int* const d_segments,
								const unsigned int* const d_countSegments,
								const float* const d_means,
								float* const d_feat,
								int numPoints,
								int numEntries,
								const FeatParams* const d_featParams)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if(idx < numPoints){
		int entry = d_segments[idx];
		if(entry >= 0){
			int entryCount = d_countSegments[entry];

			int startRow = 0;
			//covar DI
			float c11 = (d_terrain[0 * numPoints + idx] - d_means[0 * numEntries + entry]) *
							(d_terrain[0 * numPoints + idx] - d_means[0 * numEntries + entry])/entryCount;
			float c12 = (d_terrain[0 * numPoints + idx] - d_means[0 * numEntries + entry]) *
							(d_terrain[1 * numPoints + idx] - d_means[1 * numEntries + entry])/entryCount;
			float c22 = (d_terrain[1 * numPoints + idx] - d_means[1 * numEntries + entry]) *
							(d_terrain[1 * numPoints + idx] - d_means[1 * numEntries + entry])/entryCount;
			atomicAddFloat(&d_feat[0 * numEntries + entry], c11);
			atomicAddFloat(&d_feat[1 * numEntries + entry], c12);
			atomicAddFloat(&d_feat[2 * numEntries + entry], c12);
			atomicAddFloat(&d_feat[3 * numEntries + entry], c22);
			startRow += 4;
		}
	}
}

__global__ void classifySVM(const float* const d_feat,
							int* const predVal,
							int numEntries,
							int numFeat)
{

}

#endif //CAMERA_KERNELS_CU
