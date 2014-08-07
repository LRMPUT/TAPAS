
#ifndef CAMERA_KERNELS_CU
#define CAMERA_KERNELS_CU

__device__ __forceinline__ void multMat(float* d_A,
										float* d_B,
										float* d_C,
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

#endif //CAMERA_KERNELS_CU
