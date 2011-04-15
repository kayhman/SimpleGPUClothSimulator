#include "clothSimulation.h"
#include <iostream>

texture<float, 2, cudaReadModeElementType> nodesPositionTex; 

///////////////////////////////////////////
//           CUDA Kernel Code            //
///////////////////////////////////////////
 __global__ 
 void internalForcesKernel(float h,
 					float* forcesX,
 					float* forcesY,
 					float* forcesZ)
{
	const int nodeIdxI = 0;
	const int nodeIdxJ = 0; 
	const int nodeIdx = 0; 
	
	__shared__ float nX[9 * 32 * 3];
	__shared__ float nY[9 * 32 * 3];
	__shared__ float nZ[9 * 32 * 3];
	
	const float X = tex2D(nodesPositionTex, nodeIdxI, nodeIdxJ);
	const float Y = tex2D(nodesPositionTex, nodeIdxI, nodeIdxJ);
	const float Z = tex2D(nodesPositionTex, nodeIdxI, nodeIdxJ);

	for(int i = -1 ; i < 2 ; ++i)
		for(int j = -1 ; j < 2 ; ++i)
			if(i != 0 || j != 0)
			{
				nX[(i+1)*3 + (j+1)] = tex2D(nodesPositionTex, nodeIdxI+i, nodeIdxJ+j);
				nY[(i+1)*3 + (j+1)] = tex2D(nodesPositionTex, nodeIdxI+i, nodeIdxJ+j);
				nZ[(i+1)*3 + (j+1)] = tex2D(nodesPositionTex, nodeIdxI+i, nodeIdxJ+j);
		 	}
		 	
	float f = 0.;
	float fX = 0;
	float fY = 0;
	float fZ = 0;

	//non diag forces
	f =  (nX[1] - X) * (nX[1] - X);
	f +=  (nY[1] - Y) * (nY[1] - Y);
	f +=  (nZ[1] - Z) * (nZ[1] - Z);
	f = sqrt(f);
	fX += (f - h) * (nX[1] - X) / f;
	fY += (f - h) * (nY[1] * Y) / f;
	fZ += (f - h) * (nZ[1] - Z) / f;

	f =   (nX[3] - X) * (nX[3] - X);
	f +=  (nY[3] - Y) * (nY[3] - Y);
	f +=  (nZ[3] - Z) * (nZ[3] - Z);
	f = sqrt(f);
	fX += (f - h) * (nX[3] - X) / f;
	fY += (f - h) * (nY[3] - Y) / f;
	fZ += (f - h) * (nZ[3] - Z) / f;

	f =   (nX[4] - X) * (nX[4] - X);
	f +=  (nY[4] - Y) * (nY[4] - Y);
	f +=  (nZ[4] - Z) * (nZ[4] - Z);
	f = sqrt(f);
	fX += (f - h) * (nX[4] - X) / f;
	fY += (f - h) * (nY[4] - Y) / f;
	fZ += (f - h) * (nZ[4] - Z) / f;

	f =   (nX[6] - X) * (nX[6] - X);
	f +=  (nY[6] - Y) * (nY[6] - Y);
	f +=  (nZ[6] - Z) * (nZ[6] - Z);
	f = sqrt(f);
	fX += (f - h) * (nX[6] - X) / f;
	fY += (f - h) * (nY[6] - Y) / f;
	fZ += (f - h) * (nZ[6] - Z) / f;


	// diag forces
	float sqrt2xH = sqrt(2.0) * h;
	f =   (nX[0] - X) * (nX[0] - X);
	f +=  (nY[0] - Y) * (nY[0] - Y);
	f +=  (nZ[0] - Z) * (nZ[0] - Z);
	f = sqrt(f);
	fX += (f - sqrt2xH) * (nX[0] - X) / f;
	fY += (f - sqrt2xH) * (nY[0] - Y) / f;
	fZ += (f - sqrt2xH) * (nZ[0] - Z) / f;
	
	f =   (nX[2] - X) * (nX[2] - X);
	f +=  (nY[2] - Y) * (nY[2] - Y);
	f +=  (nZ[2] - Z) * (nZ[2] - Z);
	f = sqrt(f);
	fX += (f - sqrt2xH) * (nX[2] - X) / f;
	fY += (f - sqrt2xH) * (nY[2] - Y) / f;
	fZ += (f - sqrt2xH) * (nZ[2] - Z) / f;
	
	f =   (nX[5] - X) * (nX[5] - X);
	f +=  (nY[5] - Y) * (nY[5] - Y);
	f +=  (nZ[5] - Z) * (nZ[5] - Z);
	f = sqrt(f);
	fX += (f - sqrt2xH) * (nX[5] - X) / f;
	fY += (f - sqrt2xH) * (nY[5] - Y) / f;
	fZ += (f - sqrt2xH) * (nZ[5] - Z) / f;

	f =   (nX[7] - X) * (nX[7] - X);
	f +=  (nY[7] - Y) * (nY[7] - Y);
	f +=  (nZ[7] - Z) * (nZ[7] - Z);
	f = sqrt(f);
	fX += (f - sqrt2xH) * (nX[7] - X) / f;
	fY += (f - sqrt2xH) * (nY[7] - Y) / f;
	fZ += (f - sqrt2xH) * (nZ[7] - Z) / f;

	forcesX[nodeIdx] = fX;
	forcesY[nodeIdx] = fY;
	forcesZ[nodeIdx] = fZ;
 }
 
 
///////////////////////////////////////////
//             C++ Class Code            //
///////////////////////////////////////////
ClothSimulation::ClothSimulation(int sizeX, int sizeY) :
sizeX(sizeX),
sizeY(sizeY),
nbNodes(sizeX*sizeY),
devNodePos(NULL),
devF(NULL),
k(0.),
refLengthX(0.),
refLengthY(0.),
refLengthZ(0.),
refLengthDiagX(0.),
refLengthDiagY(0.),
refLengthDiagZ(0.)
{
	cudaError_t error;
	
	error = cudaMalloc (&devNodePos, sizeX * sizeY * sizeof(float3));
	
	error = cudaMalloc (&devF, sizeX * sizeY * sizeof(float3));
	
	std::cout << cudaGetErrorString(error) << std::endl;
	
	cudaMemset (&devNodePos, 0, sizeX * sizeY * sizeof(float3));
	
	cudaMemset (&devF, 0, sizeX * sizeY * sizeof(float3));
	
	///////////////////////////////////////
	//             Bind texture          //
	///////////////////////////////////////
	cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<float3>();
	cudaBindTexture2D(0, &nodesPositionTex, devNodePos, &channelDesc, sizeX, sizeY, sizeX * sizeof(float3));
	
}

ClothSimulation::~ClothSimulation()
{
	cudaFree(devNodePos);
	
	cudaFree(devF);
}

void ClothSimulation::init(float x0, float y0, float z0, 
float lx, float ly,
float k)
{
	this->k = k;
	
	
	float dx = lx / sizeX;
	float dy = ly / sizeY;

	for(int i = 0 ; i < sizeX ; ++i)
		for(int j = 0 ; j < sizeY ; ++j)
		{
			float x = x0 + i *dx;
			float y = y0 + j *dy;
			
			nodePosition.push_back(x*0. + 0.1);
			nodePosition.push_back(y*0. + 0.2);
			nodePosition.push_back(z0*0 + 0.3);
		}
		
	std::cout << "check :" << nodePosition.size() << " " << sizeX * sizeY << std::endl;
	transfertToGpu();
}

void ClothSimulation::computeInternalForces()
{
	dim3 nBlocks(4, 4);
	
	int nbGridCells = nbNodes / (nBlocks.x * nBlocks.y);
	
	int gridSize = sqrt((double)nbGridCells);
	
	dim3 blockSize(gridSize, gridSize, 0);
	
	std::cout << "use gridSize : " << gridSize << std::endl;
	
	cudaEvent_t start, stop;
	float time;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	
	cudaEventRecord( start, 0 );
	float h = 0.001;
	float* fx = NULL;
	float* fy = NULL;
	float* fz = NULL;
	internalForcesKernel <<< nBlocks, blockSize >>> (h, fx, fy, fz);
	
	cudaEventRecord( stop, 0 );
	cudaEventSynchronize( stop );
	
	cudaEventElapsedTime( &time, start, stop );
	cudaEventDestroy( start );
	cudaEventDestroy( stop );
	
	std::cout << "computeInternalForces time : " << time << " ms" << std::endl; 
}

void ClothSimulation::computeContactForces()
{

}

void ClothSimulation::integrate()
{

}

std::vector<float>&	ClothSimulation::getNodePosition()
{
	return nodePosition;
}

void ClothSimulation::transfertToGpu()
{
	cudaMemcpy (devNodePos, &(nodePosition[0]), sizeX * sizeY * sizeof(float3), cudaMemcpyHostToDevice);
}

void ClothSimulation::transfertFromGpu()
{
	cudaMemset (devF, 0, sizeX * sizeY * sizeof(float3));
	cudaError_t error;

	error = cudaMemcpy ( &(nodePosition[0]), devF, sizeX * sizeY * sizeof(float3), cudaMemcpyDeviceToHost);
 	
 	std::cout << cudaGetErrorString(error) << std::endl;
}