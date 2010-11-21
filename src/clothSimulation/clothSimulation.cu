#include "clothSimulation.h"
#include <iostream>
 
///////////////////////////////////////////
//           CUDA Kernel Code            //
///////////////////////////////////////////
 __global__ void internalForcesKernel(float* X, float * Y, float* Z,
 float* fX,
 float* fY,
 float* fZ,
 const float k,
 const int sizeX,
 const float refLengthX,
 const float refLengthY,
 const float refLengthZ,
 const float refLengthDiagX,
 const float refLengthDiagY,
 const float refLengthDiagZ 
 )
{
	const int nodeI = blockIdx.x * blockDim.x + threadIdx.x;
	const int nodeJ = blockIdx.y * blockDim.y + threadIdx.y;
/*
	const float x = X[nodeI * sizeX + nodeJ];
	const float y = Y[nodeI * sizeX + nodeJ];
	const float z = Z[nodeI * sizeX + nodeJ];

	
	float fx = 0.0;
	float fy = 0.0;
	float fz = 0.0;
  	for(int i = -1 ; i < 1 ; ++i)
	  	for(int j = -1 ; j < 1 ; ++j)
		  	if(i != 0 && j != 0)
			  	{
			  		const int neighboursI = nodeI - 1 + i * 2 ;
			  		const int neighboursJ = nodeJ - 1 + j * 2 ;
			  		
			  		const float nx = X[neighboursI * sizeX + neighboursJ];
			  		const float ny = Y[neighboursI * sizeX + neighboursJ];
			  		const float nz = Z[neighboursI * sizeX + neighboursJ];
			  		
			  		if(i == 0 || j == 0)
			  		{
				  		fx += k * (fabs(nx - x) - refLengthX) ;
				  		fy += k * (fabs(ny - y) - refLengthY) ;
				  		fz += k * (fabs(nz - z) - refLengthZ) ;
			  		}
			  		else
			  		{
			  			fx += k * (fabs(nx - x) - refLengthDiagX) ;
				  		fy += k * (fabs(ny - y) - refLengthDiagY) ;
				  		fz += k * (fabs(nz - z) - refLengthDiagZ) ;
			  		}
			  	}
*/
	fX[nodeI * sizeX + nodeJ] = 0.;//fx;
	fY[nodeI * sizeX + nodeJ] = 1.;//fy;
	fZ[nodeI * sizeX + nodeJ] = 2.;//fz;

}
 
 
///////////////////////////////////////////
//             C++ Class Code            //
///////////////////////////////////////////
ClothSimulation::ClothSimulation(int sizeX, int sizeY) :
sizeX(sizeX),
sizeY(sizeY),
nbNodes(sizeX*sizeY),
devNodeX(NULL),
devNodeY(NULL),
devNodeZ(NULL),
devFx(NULL),
devFy(NULL),
devFz(NULL),
k(0.),
b(0.),
refLengthX(0.),
refLengthY(0.),
refLengthZ(0.),
refLengthDiagX(0.),
refLengthDiagY(0.),
refLengthDiagZ(0.)
{
	cudaError_t error;
	
	cudaMalloc (&devNodeX, sizeX * sizeY * sizeof(float));
	cudaMalloc (&devNodeY, sizeX * sizeY * sizeof(float));
	cudaMalloc (&devNodeZ, sizeX * sizeY * sizeof(float));
	
	cudaMalloc (&devFx, sizeX * sizeY * sizeof(float));
	error = cudaMalloc (&devFy, sizeX * sizeY * sizeof(float));
	cudaMalloc (&devFz, sizeX * sizeY * sizeof(float));
	
	
	std::cout << cudaGetErrorString(error) << std::endl;
	
	cudaMemset (&devNodeX, 0, sizeX * sizeY * sizeof(float));
	cudaMemset (&devNodeY, 0, sizeX * sizeY * sizeof(float));
	cudaMemset (&devNodeZ, 0, sizeX * sizeY * sizeof(float));
	
	cudaMemset (&devFx, 0, sizeX * sizeY * sizeof(float));
	cudaMemset (&devFy, 0, sizeX * sizeY * sizeof(float));
	cudaMemset (&devFz, 0, sizeX * sizeY * sizeof(float));
	
}

ClothSimulation::~ClothSimulation()
{
	cudaFree(devNodeX);
	cudaFree(devNodeY);
	cudaFree(devNodeZ);
	
	cudaFree(devFx);
	cudaFree(devFy);
	cudaFree(devFz);
}

void ClothSimulation::init(float x0, float y0, float z0, 
float lx, float ly,
float k,
float b)
{
	this->k = k;
	this->b = b;
	
	
	float dx = lx / sizeX;
	float dy = ly / sizeY;

	for(int i = 0 ; i < sizeX ; ++i)
		for(int j = 0 ; j < sizeY ; ++j)
		{
			float x = x0 + i *dx;
			float y = y0 + i *dy;
			
			nodeX.push_back(x);
			nodeY.push_back(y);
			nodeZ.push_back(z0);
		}
		
	std::cout << "check :" << nodeX.size() << " " << sizeX * sizeY << std::endl;
	transfertToGpu();
}

void ClothSimulation::computeInternalForces()
{
	dim3 nBlocks(4, 4);
	
	int nbGridCells = nbNodes / (nBlocks.x * nBlocks.y);
	
	int gridSize = sqrt(nbGridCells);
	
	dim3 blockSize(gridSize, gridSize, 0);
	
	std::cout << "use gridSize : " << gridSize << std::endl;
	
	cudaEvent_t start, stop;
	float time;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	
	cudaEventRecord( start, 0 );
	internalForcesKernel <<< nBlocks, blockSize >>> (devNodeX, devNodeY, devNodeZ,
	 devFx,
	 devFy,
	 devFz,
	 k,
	 sizeX,
	 refLengthX,
	 refLengthY,
	 refLengthZ,
	 refLengthDiagX,
	 refLengthDiagY,
	 refLengthDiagZ);
	
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

std::vector<float>&	ClothSimulation::getNodeX()
{
	return nodeX;
}

std::vector<float>&	ClothSimulation::getNodeY()
{
	return nodeY;
}

std::vector<float>&	ClothSimulation::getNodeZ()
{
	return nodeZ;
}
	
void ClothSimulation::transfertToGpu()
{
	cudaMemcpy (devNodeX, &(nodeX[0]), sizeX * sizeY * sizeof(float), cudaMemcpyHostToDevice);
 	cudaMemcpy (devNodeY, &(nodeY[0]), sizeX * sizeY * sizeof(float), cudaMemcpyHostToDevice);
 	cudaMemcpy (devNodeZ, &(nodeZ[0]), sizeX * sizeY * sizeof(float), cudaMemcpyHostToDevice);
}

void ClothSimulation::transfertFromGpu()
{
	cudaMemset (&devFx, 0, sizeX * sizeY * sizeof(float));
	cudaMemset (&devFy, 0, sizeX * sizeY * sizeof(float));
	cudaMemset (&devFz, 0, sizeX * sizeY * sizeof(float));
	
	cudaError_t error;

	cudaMemcpy ( &(nodeX[0]), devFx, sizeX * sizeY * sizeof(float), cudaMemcpyDeviceToHost);
 	error = cudaMemcpy ( &(nodeY[0]), devFy, sizeX * sizeY * sizeof(float), cudaMemcpyDeviceToHost);
 	cudaMemcpy ( &(nodeZ[0]), devFz, sizeX * sizeY * sizeof(float), cudaMemcpyDeviceToHost);
 	
 	std::cout << cudaGetErrorString(error) << std::endl;
}