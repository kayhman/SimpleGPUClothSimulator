#pragma once
#include <vector>
#include "triangle.h"

class ClothSimulation
{
private:
	std::vector<float> nodeX;
	std::vector<float> nodeY;
	std::vector<float> nodeZ;
	
	std::vector<Triangle> triangles;

	int sizeX;
	int sizeY;
	int nbNodes;

	//device variables
	float* devNodeX;
	float* devNodeY;
	float* devNodeZ;
	
	float* devNodeUx;
	float* devNodeUy;
	float* devNodeUz;

	float* devFx;
	float* devFy;
	float* devFz;

	float k;
	float b;
	float refLength;
	float mass;

public:
	ClothSimulation();
	~ClothSimulation();

	void init(float x0, float y0, float z0, 
			float lx, float ly,
			float elementSize,
			float k, float b);
	void computeInternalForces();
	void handleCollision(int collisionType);
	void integrate();


	//Accessors
	std::vector<float>&	getNodeX();
	std::vector<float>&	getNodeY();
	std::vector<float>&	getNodeZ();
	int getSizeX();
	int getSizeY();
	

private:
	void allocateGPUMem();
	void transfertToGpu();
public:
	void transfertFromGpu();

};
