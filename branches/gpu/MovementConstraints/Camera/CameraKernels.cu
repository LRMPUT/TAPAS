
#ifndef CAMERA_KERNELS_CU
#define CAMERA_KERNELS_CU

#include "CameraCuda.h"

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


__device__ float atomicAddOwn(float* address, float val)
{
    unsigned int* address_as_uint =
                              (unsigned int*)address;
    unsigned long long int old = *address_as_uint, assumed;
    do {
        assumed = old;
        old = atomicCAS(address_as_uint, assumed,
                        __float_as_int(val +
                               __int_as_float(assumed)));
    } while (assumed != old);
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

	if(idxX < numCols && idxY < numRows){
		int segment = d_segments[idx1d];
		if(segment >= 0){
			atomicInc(&d_countSegments[segment], 0xffffffff);
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

__global__ void countTerrainPixels(const int* const d_segments,
								unsigned int* const d_countSegments,
								int numPoints)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if(idx < numPoints){
		int segment = d_segments[idx];
		if(segment >= 0){
			atomicInc(&d_countSegments[segment], 0xffffffff);
		}
	}
}

__global__ void compImageHistHSV(const unsigned char* const d_h,
								const unsigned char* const d_s,
								const unsigned char* const d_v,
								const unsigned int* const d_countSegments,
								float* const d_feat,
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
			int entryCount = d_countSegments[entry];

			int startRow = 0;

			//histHS
			int hBin = (d_h[idx1d] - d_featParams->histHRangeMin) /
					(d_featParams->histHRangeMax - d_featParams->histHRangeMin) * d_featParams->histHLen;
			int sBin = (d_s[idx1d] - d_featParams->histSRangeMin) /
							(d_featParams->histSRangeMax - d_featParams->histSRangeMin) * d_featParams->histSLen;
			atomicAddOwn(&d_feat[(startRow + sBin * d_featParams->histHLen + hBin)* numEntries + entry], 1.0/entryCount);
			startRow += d_featParams->histHLen * d_featParams->histSLen;

			//histV
			int vBin  = (d_v[idx1d] - d_featParams->histVRangeMin) /
					(d_featParams->histVRangeMax - d_featParams->histVRangeMin) * d_featParams->histVLen;
			atomicAddOwn(&d_feat[(startRow + vBin) * numEntries + entry], 1.0/entryCount);
			startRow += d_featParams->histVLen;
		}
	}
}

__global__ void compImageMeanHSV(const unsigned char* const d_h,
								const unsigned char* const d_s,
								const unsigned char* const d_v,
								const unsigned int* const d_countSegments,
								float* const d_feat,
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
			int entryCount = d_countSegments[entry];

			int startRow = 0;
			//meanHSV
			atomicAddOwn(&d_feat[(startRow + 0) * numEntries + entry], (float)d_h[idx1d]/entryCount);
			atomicAddOwn(&d_feat[(startRow + 1) * numEntries + entry], (float)d_s[idx1d]/entryCount);
			atomicAddOwn(&d_feat[(startRow + 2) * numEntries + entry], (float)d_v[idx1d]/entryCount);
			startRow += 3;
		}
	}
}

__global__ void compImageCovarHSV(const unsigned char* const d_h,
								const unsigned char* const d_s,
								const unsigned char* const d_v,
								const unsigned int* const d_countSegments,
								const float* const d_means,
								float* const d_feat,
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
			int entryCount = d_countSegments[entry];

			int startRow = 0;
			//covar HSV
			float c11 = ((float)d_h[idx1d] -  d_means[0 * numEntries + entry])*((float)d_h[idx1d] -  d_means[0 * numEntries + entry])/entryCount;
			float c12 = ((float)d_h[idx1d] -  d_means[0 * numEntries + entry])*((float)d_s[idx1d] -  d_means[1 * numEntries + entry])/entryCount;
			float c13 = ((float)d_h[idx1d] -  d_means[0 * numEntries + entry])*((float)d_v[idx1d] -  d_means[2 * numEntries + entry])/entryCount;
			float c22 = ((float)d_s[idx1d] -  d_means[1 * numEntries + entry])*((float)d_s[idx1d] -  d_means[1 * numEntries + entry])/entryCount;
			float c23 = ((float)d_s[idx1d] -  d_means[1 * numEntries + entry])*((float)d_v[idx1d] -  d_means[2 * numEntries + entry])/entryCount;
			float c33 = ((float)d_v[idx1d] -  d_means[2 * numEntries + entry])*((float)d_v[idx1d] -  d_means[2 * numEntries + entry])/entryCount;
			atomicAddOwn(&d_feat[(startRow + 0) * numEntries + entry], c11);
			atomicAddOwn(&d_feat[(startRow + 1) * numEntries + entry], c12);
			atomicAddOwn(&d_feat[(startRow + 2) * numEntries + entry], c13);
			atomicAddOwn(&d_feat[(startRow + 3) * numEntries + entry], c12);
			atomicAddOwn(&d_feat[(startRow + 4) * numEntries + entry], c22);
			atomicAddOwn(&d_feat[(startRow + 5) * numEntries + entry], c23);
			atomicAddOwn(&d_feat[(startRow + 6) * numEntries + entry], c12);
			atomicAddOwn(&d_feat[(startRow + 7) * numEntries + entry], c23);
			atomicAddOwn(&d_feat[(startRow + 8) * numEntries + entry], c33);
			startRow += 9;
		}
	}
}

__global__ void compTerrainHistDI(const int* const d_terrain,
								const int* const d_segments,
								const unsigned int* const countSegments,
								float* const d_feat,
								int numPoints,
								int numEntries,
								const FeatParams* const d_featParams)
{

}

__global__ void compTerrainMeanDI(const int* const d_terrain,
								const int* const d_segments,
								const unsigned int* const countSegments,
								float* const d_feat,
								int numPoints,
								int numEntries,
								const FeatParams* const d_featParams)
{

}

__global__ void compTerrainCovarDI(const int* const d_terrain,
								const int* const d_segments,
								const unsigned int* const countSegments,
								float* const d_feat,
								int numPoints,
								int numEntries,
								const FeatParams* const d_featParams)
{

}

__global__ void classifySVM(const float* const d_feat,
							int* const predVal,
							int numEntries,
							int numFeat)
{

}

#endif //CAMERA_KERNELS_CU
