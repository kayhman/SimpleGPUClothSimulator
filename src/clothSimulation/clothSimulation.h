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

	float* devFx;
	float* devFy;
	float* devFz;

	float k;
	float b;
	float refLengthX;
	float refLengthY;
	float refLengthZ;
	float refLengthDiagX;
	float refLengthDiagY;
	float refLengthDiagZ;

public:
	ClothSimulation(int sizeX, int sizeY);
	~ClothSimulation();

	void init(float x0, float y0, float z0, float lx, float ly,
			float k, float b);
	void computeInternalForces();
	void computeContactForces();
	void integrate();

	//Accessors
	std::vector<float>&	getNodeX();
	std::vector<float>&	getNodeY();
	std::vector<float>&	getNodeZ();
	int getSizeX();
	int getSizeY();
	

private:
	void transfertToGpu();
public:
	void transfertFromGpu();

};
