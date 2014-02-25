#set(CMAKE_DISABLE_SOURCE_CHANGES ON)
#set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

# Project name is not mandatory, but you should use it
project(MEKA_BRIDGE)

# States that CMake required version must be greater than 2.6
cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_FLAGS "-Wall -std=c++0x")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Appends the cmake/modules path inside the MAKE_MODULE_PATH variable which stores the
# directories of additional CMake modules (ie. MacroOutOfSourceBuild.cmake):
#set(CMAKE_MODULE_PATH ${oo_dmp_bbo_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})

#include_directories(${CMAKE_SOURCE_DIR}/../cpp/include)
#link_directories(${CMAKE_SOURCE_DIR}/../cpp/lib)

#IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#  SET(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR} CACHE PATH "Comment" FORCE)
#ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)


#include_directories(${CMAKE_SOURCE_DIR}/../thirdparty/lwpr/include)    
#get_property(inc_dirs DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
#find_file(HAVE_LWPR_HH lwpr.hh PATHS ${inc_dirs})

# Boost stuff
#find_package( Boost 1.34 COMPONENTS filesystem system)
#link_directories ( ${Boost_LIBRARY_DIRS} )
#include_directories ( ${Boost_INCLUDE_DIRS} )

#add_subdirectory(src)

#option(BUILD_DOCUMENTATION "Use Doxygen to create the HTML based API documentation" ON)
#if(BUILD_DOCUMENTATION)
#  FIND_PACKAGE(Doxygen)
#  if (NOT DOXYGEN_FOUND)
#    message(FATAL_ERROR 
#      "Doxygen is needed to build the documentation. Please install it correctly")
#  endif()
  #-- Configure the Template Doxyfile for our specific project
#  configure_file(${CMAKE_SOURCE_DIR}/docs/Doxyfile.in 
#                 ${PROJECT_BINARY_DIR}/Doxyfile  @ONLY IMMEDIATE)
  #-- Add a custom target to run Doxygen when ever the project is built
#  add_custom_target (Docs  
#  										COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
#  										SOURCES ${PROJECT_BINARY_DIR}/Doxyfile)
  # IF you do NOT want the documentation to be generated EVERY time you build the project
  # then leave out the 'ALL' keyword from the above command.
#endif()

#set(DMPCORE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../cpp/include)
#set(DMPCORE_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/../cpp/lib)
#link_directories(${DMPCORE_LIBRARY_DIR})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
find_package(RTAI REQUIRED)
find_package(M3 COMPONENTS HARDWARE ROBOTS CHAINS TOOLBOX SHARED_MEM REQUIRED)
find_package(M3rt REQUIRED)
find_package(Boost COMPONENTS filesystem system serialization REQUIRED)

SET(LIBS ${RTAI_LIBRARIES} ${M3_LIBRARIES} ${M3RT_LIBRARIES} pthread dmp dynamicalsystems functionapproximators ${Boost_LIBRARIES})
include_directories(${M3RT_INCLUDE_DIR} ${RTAI_INCLUDE_DIR} ${M3_INCLUDE_DIR} ${THREADS_INCLUDE_DIR})

add_executable(demoMekaShmTrj src/MekaShmTrj.cpp)
target_link_libraries(demoMekaShmTrj ${LIBS})

add_executable(demoMekaShmDmp src/MekaShmDmp.cpp)
target_link_libraries(demoMekaShmDmp ${LIBS})
install(TARGETS demoMekaShmDmp DESTINATION ${CMAKE_SOURCE_DIR}/bin)
