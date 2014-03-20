#set(CMAKE_DISABLE_SOURCE_CHANGES ON)
#set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

# Project name is not mandatory, but you should use it
project(MEKA_DMP_BRIDGE)

# States that CMake required version must be greater than 2.6
cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_FLAGS "-Wall -std=c++0x")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
find_package(RTAI REQUIRED)
find_package(M3 COMPONENTS HARDWARE ROBOTS CHAINS TOOLBOX SHARED_MEM REQUIRED)
find_package(M3rt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS filesystem system serialization REQUIRED)
find_package(orocos_kdl REQUIRED)

SET(LIBS ${RTAI_LIBRARIES} ${M3_LIBRARIES} ${M3RT_LIBRARIES} ${orocos_kdl_LIBRARIES} pthread dmp dynamicalsystems functionapproximators ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES} meka_kinematics)
include_directories(${M3RT_INCLUDE_DIR} ${RTAI_INCLUDE_DIR} ${M3_INCLUDE_DIR} ${THREADS_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS})

#add_executable(demoMekaShmTrj src/MekaShmTrj.cpp)
#target_link_libraries(demoMekaShmTrj ${LIBS})

#add_executable(demoMekaShmDmp src/MekaShmDmp.cpp)
#target_link_libraries(demoMekaShmDmp ${LIBS})
#install(TARGETS demoMekaShmDmp DESTINATION ${CMAKE_SOURCE_DIR}/bin)

add_executable(demoMekaShmDmpGen src/MekaShmDmpGen.cpp)
target_link_libraries(demoMekaShmDmpGen ${LIBS})
install(TARGETS demoMekaShmDmpGen DESTINATION ${CMAKE_SOURCE_DIR}/bin)

#add_executable(testDmp src/testDmp.cpp)
#target_link_libraries(testDmp ${LIBS})
#install(TARGETS testDmp DESTINATION ${CMAKE_SOURCE_DIR}/bin)
