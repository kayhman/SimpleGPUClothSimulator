set(CUDA_NVCC_FLAGS -arch sm_11)
cuda_add_library(clothSimulation clothSimulation.h clothSimulation.cu)
