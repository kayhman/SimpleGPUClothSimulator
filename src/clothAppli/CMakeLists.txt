include_directories(../clothSimulation)
include_directories(../clothDisplay)

add_executable(clothSimulator clothSimulator.cpp)
target_link_libraries(clothSimulator clothSimulation)
