find_package(CUDA)

set(CUDA_NVCC_FLAGS -Xptxas -v)

add_subdirectory(clothSimulation)
add_subdirectory(clothDisplay)
add_subdirectory(clothAppli)
