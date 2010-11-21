#include <iostream>
#include "clothSimulation.h"


int main(int argc, char* argv[])
{

	ClothSimulation clothSimulation(512*10, 512*10);

	clothSimulation.init(0., 0., 0.,
						0.15, 0.15,
						1e3, 1e3);

	clothSimulation.computeInternalForces();

	clothSimulation.transfertFromGpu();

	for(int i = 0 ; i < 512 * 10 ; ++i)
	{
		std::cout << clothSimulation.getNodeY()[i] << std::endl;
	}

	std::cout << "welcome into our cloth simulator. Please select an item." << std::endl;
	return 0;
}
