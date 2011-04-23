#pragma once
#include <vector>
#include "triangle.h"

class ClothSimulation
{
private:
	std::vector<float> nodeX;
	std::vector<float> nodeY;
	std::vector<float> nodeZ;
	
	int sizeX;
	int sizeY;
	int nbNodes;

	//device variables
	float* devNodePos;
	float* devF;

	float k;
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
			float k);
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
