#include "clothSimulation.h"
#include <iostream>
 
///////////////////////////////////////////
//           CUDA Kernel Code            //
///////////////////////////////////////////

const float gravity = 9.81f;
const float dt = 1e-3;


 __global__ void internalForcesKernel(float* X, float * Y, float* Z,
 float* fX,
 float* fY,
 float* fZ,
 const float k,
 const int sizeX,
 const float h,
 const float mass)
{
	const int nodeI = blockIdx.x * blockDim.x + threadIdx.x;
	const int nodeJ = blockIdx.y * blockDim.y + threadIdx.y;

	const float x = X[nodeI * sizeX + nodeJ];
	const float y = Y[nodeI * sizeX + nodeJ];
	const float z = Z[nodeI * sizeX + nodeJ];

	
	float fx = 0.0;
	float fy = 0.0;
	float fz = - mass * gravity;

  	for(int i = -1 ; i < 2 ; ++i)
	  	for(int j = -1 ; j < 2 ; ++j)
		  	if(! (i == 0 && j == 0) )
			  	{
			  		const int neighboursI = nodeI + i;
			  		const int neighboursJ = nodeJ + j;
			  		
			  		if(neighboursI > 0 && neighboursJ > 0 &&
			  		   neighboursI < sizeX && neighboursJ < sizeX)
			  		{
				  		const float nx = X[neighboursI * sizeX + neighboursJ] - x;
				  		const float ny = Y[neighboursI * sizeX + neighboursJ] - y;
				  		const float nz = Z[neighboursI * sizeX + neighboursJ] - z;
				  		const float norm = sqrt(nx * nx + ny * ny + nz * nz);
				  		
				  		if(norm != 0.0)
				  		{
					  		if(i == 0 || j == 0)
					  		{
					  			const float diff = norm - h;
						  		fx += k * diff * nx / norm;
						  		fy += k * diff * ny / norm;
						  		fz += k * diff * nz / norm;
					  		}
					  		else
					  		{
					  			const float diff = norm - sqrt(2.0) * h;
						  		fx += k * diff * nx / norm;
						  		fy += k * diff * ny / norm;
						  		fz += k * diff * nz / norm;
					  		
					  		}
					  	}
				  	}
			  	}

	fX[nodeI * sizeX + nodeJ] = fx;
	fY[nodeI * sizeX + nodeJ] = fy;
	fZ[nodeI * sizeX + nodeJ] = fz;
}

__global__ void integrateNodePosition(float* X, float * Y, float* Z,
float* Ux, float * Uy, float* Uz,
 float* fX,
 float* fY,
 float* fZ,
 const int sizeX,
 const float mass)
{
	const int nodeI = blockIdx.x * blockDim.x + threadIdx.x;
	const int nodeJ = blockIdx.y * blockDim.y + threadIdx.y;

	Ux[nodeI * sizeX + nodeJ] += dt * (fX[nodeI * sizeX + nodeJ] - Ux[nodeI * sizeX + nodeJ] * 0.02)  / mass;
	Uy[nodeI * sizeX + nodeJ] += dt * (fY[nodeI * sizeX + nodeJ] - Uy[nodeI * sizeX + nodeJ] * 0.02)  / mass;
	Uz[nodeI * sizeX + nodeJ] += dt * (fZ[nodeI * sizeX + nodeJ] - Uz[nodeI * sizeX + nodeJ] * 0.02)  / mass;


	X[nodeI * sizeX + nodeJ] += dt * Ux[nodeI * sizeX + nodeJ];
	Y[nodeI * sizeX + nodeJ] += dt * Uy[nodeI * sizeX + nodeJ];
	Z[nodeI * sizeX + nodeJ] += dt * Uz[nodeI * sizeX + nodeJ];
	
	
}

__global__ void handleCollisionWithDisk(float* X, float * Y, float* Z,
 float* fX,
 float* fY,
 float* fZ,
 const int sizeX,
 const float mass)
{
	const int nodeI = blockIdx.x * blockDim.x + threadIdx.x;
	const int nodeJ = blockIdx.y * blockDim.y + threadIdx.y;

	const float diskZ = 0.0f;
	const float diskRadius = 1.75f;
	
	const float diskCenterX = 0.f;
	const float diskCenterY = 0.f;
	
	const float x = X[nodeI * sizeX + nodeJ];
	const float y = Y[nodeI * sizeX + nodeJ];
	const float z = Z[nodeI * sizeX + nodeJ];

	if(z < diskZ && z > diskZ - 0.01)
	{
		const float distToCenter = sqrt( (x - diskCenterX) * (x - diskCenterX) + (y - diskCenterY) * (y- diskCenterY) );
		
		if(distToCenter < diskRadius)
		{
			fZ[nodeI * sizeX + nodeJ] += (diskZ - z) * mass / (dt*dt);
		}
	 }
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
devNodeUx(NULL),
devNodeUy(NULL),
devNodeUz(NULL),
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
refLengthDiagZ(0.),
mass(0.)
{
	cudaError_t error;
	
	cudaMalloc (&devNodeX, sizeX * sizeY * sizeof(float));
	cudaMalloc (&devNodeY, sizeX * sizeY * sizeof(float));
	cudaMalloc (&devNodeZ, sizeX * sizeY * sizeof(float));
	
	cudaMalloc (&devNodeUx, sizeX * sizeY * sizeof(float));
	cudaMalloc (&devNodeUy, sizeX * sizeY * sizeof(float));
	cudaMalloc (&devNodeUz, sizeX * sizeY * sizeof(float));
	
	
	cudaMalloc (&devFx, sizeX * sizeY * sizeof(float));
	cudaMalloc (&devFy, sizeX * sizeY * sizeof(float));
	cudaMalloc (&devFz, sizeX * sizeY * sizeof(float));
	
	
	std::cout << cudaGetErrorString(error) << std::endl;
	
	cudaMemset (devNodeX, 0, sizeX * sizeY * sizeof(float));
	cudaMemset (devNodeY, 0, sizeX * sizeY * sizeof(float));
	cudaMemset (devNodeZ, 0, sizeX * sizeY * sizeof(float));
	
	cudaMemset (devNodeUx, 0, sizeX * sizeY * sizeof(float));
	cudaMemset (devNodeUy, 0, sizeX * sizeY * sizeof(float));
	cudaMemset (devNodeUz, 0, sizeX * sizeY * sizeof(float));
		
	cudaMemset (devFx, 0, sizeX * sizeY * sizeof(float));
	cudaMemset (devFy, 0, sizeX * sizeY * sizeof(float));
	cudaMemset (devFz, 0, sizeX * sizeY * sizeof(float));
}

ClothSimulation::~ClothSimulation()
{
	cudaFree(devNodeX);
	cudaFree(devNodeY);
	cudaFree(devNodeZ);

	cudaFree(devNodeUx);
	cudaFree(devNodeUy);
	cudaFree(devNodeUz);
	
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
	
	refLengthX = dx;
	refLengthY = dy;
	refLengthZ = 0.;

	refLengthDiagX = sqrt(dx * dx + dy * dy);
	refLengthDiagY = sqrt(dx * dx + dy * dy);
	refLengthDiagZ = sqrt(dx * dx + dy * dy);

	for(int i = 0 ; i < sizeX ; ++i)
		for(int j = 0 ; j < sizeY ; ++j)
		{
			float x = x0 + i *dx;
			float y = y0 + j *dy;
			
			nodeX.push_back(x);
			nodeY.push_back(y);
			nodeZ.push_back(z0);
		}

	mass = 3.0 * (lx*ly) / (sizeX*sizeY);
		
	std::cout << "check :" << nodeX.size() << " " << sizeX * sizeY << std::endl;
	transfertToGpu();
}

void ClothSimulation::computeInternalForces()
{
	dim3 nBlocks(4, 4);
	
	int nbGridCells = nbNodes / (nBlocks.x * nBlocks.y);
	
	int gridSize = sqrt((double)nbGridCells);
	
	dim3 blockSize(gridSize, gridSize);
	
	std::cout << "use gridSize : " << gridSize << std::endl;
	
	cudaEvent_t start, stop;
	float time;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	
	cudaEventRecord( start, 0 );
	internalForcesKernel <<< blockSize, nBlocks >>> (devNodeX, devNodeY, devNodeZ,
	 devFx,
	 devFy,
	 devFz,
	 k,
	 sizeX,
	 refLengthX,
	 mass);
	 	
	cudaEventRecord( stop, 0 );
	cudaEventSynchronize( stop );
	
	cudaEventElapsedTime( &time, start, stop );
	cudaEventDestroy( start );
	cudaEventDestroy( stop );
	
	std::cout << "computeInternalForces time : " << time << " ms : ref -> " << refLengthX << std::endl; 
}

void ClothSimulation::handleCollision()
{
	dim3 nBlocks(4, 4);
	
	int nbGridCells = nbNodes / (nBlocks.x * nBlocks.y);
	
	int gridSize = sqrt((double)nbGridCells);
	
	dim3 blockSize(gridSize, gridSize);
	
	std::cout << "use gridSize : " << gridSize << std::endl;
	
	cudaEvent_t start, stop;
	float time;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	
	cudaEventRecord( start, 0 );
	handleCollisionWithDisk <<< blockSize, nBlocks >>> (devNodeX, devNodeY, devNodeZ,
	 devFx,
	 devFy,
	 devFz,
	 sizeX,
	 mass);
	 	
	cudaEventRecord( stop, 0 );
	cudaEventSynchronize( stop );
	
	cudaEventElapsedTime( &time, start, stop );
	cudaEventDestroy( start );
	cudaEventDestroy( stop );
	
	std::cout << "collision handling time : " << time << " ms : ref -> " << refLengthX << std::endl; 
}

void ClothSimulation::computeContactForces()
{

}

void ClothSimulation::integrate()
{
	dim3 nBlocks(4, 4);
	
	int nbGridCells = nbNodes / (nBlocks.x * nBlocks.y);
	
	int gridSize = sqrt((double)nbGridCells);
	
	dim3 blockSize(gridSize, gridSize);
	
	std::cout << "use gridSize : " << gridSize << std::endl;
	
	cudaEvent_t start, stop;
	float time;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	
	cudaEventRecord( start, 0 );
	integrateNodePosition <<<  blockSize, nBlocks >>> (devNodeX, devNodeY, devNodeZ,
	devNodeUx, devNodeUy, devNodeUz,
	 devFx,
	 devFy,
	 devFz,
	 sizeX,
	 mass);
	
	cudaEventRecord( stop, 0 );
	cudaEventSynchronize( stop );
	
	cudaEventElapsedTime( &time, start, stop );
	cudaEventDestroy( start );
	cudaEventDestroy( stop );
	
	std::cout << " integrate error " << cudaGetErrorString(cudaGetLastError()) << std::endl;
	std::cout << "Integrate time : " << time << " ms" << std::endl; 
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
	
int ClothSimulation::getSizeX()
{
	return sizeX;
}

int ClothSimulation::getSizeY()
{
	return sizeY;
}
	
void ClothSimulation::transfertToGpu()
{
	cudaMemcpy (devNodeX, &(nodeX[0]), sizeX * sizeY * sizeof(float), cudaMemcpyHostToDevice);
 	cudaMemcpy (devNodeY, &(nodeY[0]), sizeX * sizeY * sizeof(float), cudaMemcpyHostToDevice);
 	cudaMemcpy (devNodeZ, &(nodeZ[0]), sizeX * sizeY * sizeof(float), cudaMemcpyHostToDevice);
}

void ClothSimulation::transfertFromGpu()
{
	cudaError_t error;
	
	//error = cudaMemset (devNodeX, 0, sizeX * sizeY * sizeof(float));
	//cudaMemset (devNodeY, 0, sizeX * sizeY * sizeof(float));
	//cudaMemset (devNodeZ, 0, sizeX * sizeY * sizeof(float));
	
	//std::cout << " 1 " << cudaGetErrorString(error) << std::endl;

	error = cudaMemcpy ( &(nodeX[0]), devNodeX, sizeX * sizeY * sizeof(float), cudaMemcpyDeviceToHost);
 	error = cudaMemcpy ( &(nodeY[0]), devNodeY, sizeX * sizeY * sizeof(float), cudaMemcpyDeviceToHost);
 	error = cudaMemcpy ( &(nodeZ[0]), devNodeZ, sizeX * sizeY * sizeof(float), cudaMemcpyDeviceToHost);
 	
 	std::cout << cudaGetErrorString(cudaGetLastError()) << std::endl;
 	
}
