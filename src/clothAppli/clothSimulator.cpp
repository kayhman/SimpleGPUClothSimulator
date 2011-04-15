#include <iostream>
#include "clothSimulation.h"


int main(int argc, char* argv[])
{

	ClothSimulation clothSimulation(512, 512);

	clothSimulation.init(0., 0., 0.,
						0.15, 0.15,
						1e3);

	clothSimulation.computeInternalForces();

//	clothSimulation.transfertFromGpu();

	for(int i = 0 ; i < 512 * 10 ; ++i)
	{
		std::cout << clothSimulation.getNodePosition()[3*i+0] << std::endl;
		std::cout << clothSimulation.getNodePosition()[3*i+1] << std::endl;
		std::cout << clothSimulation.getNodePosition()[3*i+2] << std::endl << std::endl;
	}

	std::cout << "welcome into our cloth simulator. Please select an item." << std::endl;
	return 0;
}
