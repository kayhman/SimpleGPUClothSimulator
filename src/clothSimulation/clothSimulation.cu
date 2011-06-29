#include "clothSimulation.h"
#include <iostream>
 
 #define BLOCKX 16
 #define BLOCKY 16
 
///////////////////////////////////////////
//           CUDA Kernel Code            //
///////////////////////////////////////////


 __global__ void internalForcesKernel(float* X, float * Y, float* Z,
 float* fX,
 float* fY,
 float* fZ,
 const float kStrech,
 const float kBend,
 const int sizeX,
 const int sizeY,
 const float h,
 const float mass)
{
	const float gravity = 9.81f;
	const int nodeI = blockIdx.y * blockDim.y + threadIdx.y;
	const int nodeJ = blockIdx.x * blockDim.x + threadIdx.x;

	const float x = X[nodeJ * sizeY + nodeI];
	const float y = Y[nodeJ * sizeY + nodeI];
	const float z = Z[nodeJ * sizeY + nodeI];

	
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
			  		   neighboursI < sizeY && neighboursJ < sizeX)
			  		{
				  		const float nx = X[neighboursJ * sizeY + neighboursI] - x;
				  		const float ny = Y[neighboursJ * sizeY + neighboursI] - y;
				  		const float nz = Z[neighboursJ * sizeY + neighboursI] - z;
				  		const float norm = sqrt(nx * nx + ny * ny + nz * nz);
				  		
				  		if(norm != 0.0)
				  		{
					  		if(i == 0 || j == 0) //stretching spring
					  		{
					  			const float diff = norm - h;
						  		fx += kStrech * diff * nx / norm;
						  		fy += kStrech * diff * ny / norm;
						  		fz += kStrech * diff * nz / norm;
					  		}
					  		else  //bending spring
					  		{
					  			const float diff = norm - sqrt(2.0) * h;
						  		fx += kBend * diff * nx / norm;
						  		fy += kBend * diff * ny / norm;
						  		fz += kBend * diff * nz / norm;
					  		
					  		}
					  	}
				  	}
			  	}

	fX[nodeJ * sizeY + nodeI] = fx;
	fY[nodeJ * sizeY + nodeI] = fy;
	fZ[nodeJ * sizeY + nodeI] = fz;
}

__global__ void integrateNodePosition(float* X, float * Y, float* Z,
float* Ux, float * Uy, float* Uz,
 float* fX,
 float* fY,
 float* fZ,
 const int sizeY,
 const float mass)
{
	const float dt = 1e-3;
	const int nodeI = blockIdx.y * blockDim.y + threadIdx.y;
	const int nodeJ = blockIdx.x * blockDim.x + threadIdx.x;
	
	const int idx = (blockIdx.y * gridDim.x + blockIdx.x )* (blockDim.x * blockDim.y) + threadIdx.y * blockDim.x + threadIdx.x ;
	//const int idx = nodeJ * sizeY + nodeI; 

	Ux[idx] += dt * (fX[idx] - Ux[idx] * 0.02)  / mass;
	Uy[idx] += dt * (fY[idx] - Uy[idx] * 0.02)  / mass;
	Uz[idx] += dt * (fZ[idx] - Uz[idx] * 0.02)  / mass;


	X[idx] += dt * Ux[idx];
	Y[idx] += dt * Uy[idx];
	Z[idx] += dt * Uz[idx];
	
	
}

__global__ void handleCollisionWithDisk(float* X, float * Y, float* Z,
 float* fX,
 float* fY,
 float* fZ,
 const int sizeY,
 const float mass)
{
	const int nodeI = blockIdx.y * blockDim.y + threadIdx.y;
	const int nodeJ = blockIdx.x * blockDim.x + threadIdx.x;

	//const int idx = nodeJ * sizeY + nodeI;
	const int idx = (blockIdx.y * gridDim.x + blockIdx.x )* (blockDim.x * blockDim.y) + threadIdx.y * blockDim.x + threadIdx.x ;
	
	const float diskZ = 0.0f;
	const float diskRadius = 1.75f;
	
	const float diskCenterX = 0.f;
	const float diskCenterY = 0.f;
	
	const float x = X[idx];
	const float y = Y[idx];
	const float z = Z[idx];

	if(z < diskZ && z > diskZ - 0.01)
	{
		const float distToCenter = sqrt( (x - diskCenterX) * (x - diskCenterX) + (y - diskCenterY) * (y- diskCenterY) );
		
		if(distToCenter < diskRadius)
		{
			fX[idx] = 0.;
			fY[idx] = 0.;
			fZ[idx] = 0.;
			Z[idx] = diskZ;
		}
	 }
}

__global__ void handleCollisionWithRectangle(float* X, float * Y, float* Z,
 float* fX,
 float* fY,
 float* fZ,
 const int sizeY,
 const float mass)
{
	const int nodeI = blockIdx.y * blockDim.y + threadIdx.y;
	const int nodeJ = blockIdx.x * blockDim.x + threadIdx.x;

	const float rectZ = 0.0f;
	const float minX = -1.7f;
	const float minY = -0.7f;
	const float maxX = 1.7f;
	const float maxY = 0.7f;
	
	const int idx = (blockIdx.y * gridDim.x + blockIdx.x )* (blockDim.x * blockDim.y) + threadIdx.y * blockDim.x + threadIdx.x ;
	//const int idx = nodeJ * sizeY + nodeI;
		
	const float x = X[idx];
	const float y = Y[idx];
	const float z = Z[idx];

	if(z < rectZ && z > rectZ - 0.01)
	{
		if( x > minX && x < maxX &&
		    y > minY && y < maxY)
		{
			fX[idx] = 0.;
			fY[idx] = 0.;
			fZ[idx] = 0.;
			Z[idx] = rectZ;
		}
	 }
}

 
 
///////////////////////////////////////////
//             C++ Class Code            //
///////////////////////////////////////////
ClothSimulation::ClothSimulation() :
sizeX(0),
sizeY(0),
nbNodes(0),
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
refLength(0.),
mass(0.)
{
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

void ClothSimulation::allocateGPUMem()
{
	cudaError_t error;
	
	cudaMalloc (&devNodeX, nbNodes * sizeof(float));
	cudaMalloc (&devNodeY, nbNodes * sizeof(float));
	cudaMalloc (&devNodeZ, nbNodes * sizeof(float));
	
	cudaMalloc (&devNodeUx, nbNodes * sizeof(float));
	cudaMalloc (&devNodeUy, nbNodes * sizeof(float));
	cudaMalloc (&devNodeUz, nbNodes * sizeof(float));
	
	
	cudaMalloc (&devFx, nbNodes * sizeof(float));
	cudaMalloc (&devFy, nbNodes * sizeof(float));
	cudaMalloc (&devFz, nbNodes * sizeof(float));
	
	
	std::cout << cudaGetErrorString(error) << std::endl;
	
	cudaMemset (devNodeX, 0, nbNodes * sizeof(float));
	cudaMemset (devNodeY, 0, nbNodes * sizeof(float));
	cudaMemset (devNodeZ, 0, nbNodes * sizeof(float));
	
	cudaMemset (devNodeUx, 0, nbNodes * sizeof(float));
	cudaMemset (devNodeUy, 0, nbNodes * sizeof(float));
	cudaMemset (devNodeUz, 0, nbNodes * sizeof(float));
		
	cudaMemset (devFx, 0, nbNodes * sizeof(float));
	cudaMemset (devFy, 0, nbNodes * sizeof(float));
	cudaMemset (devFz, 0, nbNodes * sizeof(float));
}

void ClothSimulation::init(float x0, float y0, float z0, 
float lx, float ly,
float elementSize,
float k,
float b)
{
	this->k = k;
	this->b = b;
	
	sizeX = BLOCKX * ((int)(lx / elementSize) / BLOCKX + 1);
	sizeY = BLOCKY * ((int)(ly / elementSize) / BLOCKY + 1);
	
	nbNodes = sizeX * sizeY;
	
	allocateGPUMem();
	
	float dx = elementSize;
	float dy = elementSize;
	
	refLength = dx;

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
		
	std::cout << "grid dim : " << sizeX  << " " <<  sizeY << std::endl;
	transfertToGpu();

}

void ClothSimulation::computeInternalForces()
{
	dim3 nBlocks(1, 64);
	dim3 blockSize(sizeX/1, sizeY/64);

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
	 10 * k,
	 sizeX,
	 sizeY,
	 refLength,
	 mass);
	 	
	cudaEventRecord( stop, 0 );
	cudaEventSynchronize( stop );
	
	cudaEventElapsedTime( &time, start, stop );
	cudaEventDestroy( start );
	cudaEventDestroy( stop );
	
	std::cout << "computeInternalForces time : " << time << " ms : ref -> " << refLength << std::endl; 
}

void ClothSimulation::handleCollision(int collisionType)
{
	dim3 nBlocks(BLOCKX, BLOCKY);
	
	dim3 blockSize(sizeX/BLOCKX, sizeY/BLOCKY);
	
	cudaEvent_t start, stop;
	float time;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	
	cudaEventRecord( start, 0 );
	if(collisionType == 0)
	{
		handleCollisionWithDisk <<< blockSize, nBlocks >>> (devNodeX, devNodeY, devNodeZ,
		 devFx,
		 devFy,
		 devFz,
		 sizeY,
		 mass);
	}	 	
	if(collisionType == 1)
	{
		handleCollisionWithRectangle <<< blockSize, nBlocks >>> (devNodeX, devNodeY, devNodeZ,
		 devFx,
		 devFy,
		 devFz,
		 sizeY,
		 mass);
	}		cudaEventRecord( stop, 0 );
	cudaEventSynchronize( stop );
	
	cudaEventElapsedTime( &time, start, stop );
	cudaEventDestroy( start );
	cudaEventDestroy( stop );
	
	std::cout << "collision handling time : " << time << " ms : ref -> " << refLength << std::endl; 
}

void ClothSimulation::integrate()
{
	dim3 nBlocks(BLOCKX, BLOCKY);
	
	dim3 blockSize(sizeX/BLOCKX, sizeY/BLOCKY);
	
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
	 sizeY,
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
	cudaMemcpy (devNodeX, &(nodeX[0]), nbNodes * sizeof(float), cudaMemcpyHostToDevice);
 	cudaMemcpy (devNodeY, &(nodeY[0]), nbNodes * sizeof(float), cudaMemcpyHostToDevice);
 	cudaMemcpy (devNodeZ, &(nodeZ[0]), nbNodes * sizeof(float), cudaMemcpyHostToDevice);
}

void ClothSimulation::transfertFromGpu()
{
	cudaError_t error;
	
	//error = cudaMemset (devNodeX, 0, nbNodes * sizeof(float));
	//cudaMemset (devNodeY, 0, nbNodes * sizeof(float));
	//cudaMemset (devNodeZ, 0, nbNodes * sizeof(float));
	
	//std::cout << " 1 " << cudaGetErrorString(error) << std::endl;

	error = cudaMemcpy ( &(nodeX[0]), devNodeX, nbNodes * sizeof(float), cudaMemcpyDeviceToHost);
 	error = cudaMemcpy ( &(nodeY[0]), devNodeY, nbNodes * sizeof(float), cudaMemcpyDeviceToHost);
 	error = cudaMemcpy ( &(nodeZ[0]), devNodeZ, nbNodes * sizeof(float), cudaMemcpyDeviceToHost);
 	
 	std::cout << cudaGetErrorString(cudaGetLastError()) << std::endl;
 	
}
