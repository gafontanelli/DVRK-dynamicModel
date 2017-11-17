# set required version
cmake_minimum_required(VERSION 2.8)

# set project name
project(dvrk_robots)

# set include drectories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/../)

# set output paths
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build_lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build_lib)
# set mbed source files
set(PSM_SRC_FILES  
	src/PSM_robot.cpp	
		
)
#MTM
set(MTM_SRC_FILES 
	src/MTM_robot.cpp	
	
)



# add custom flags to c++ compiler
set(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(utilities SHARED src/utilities.cpp)

add_library(PSM_robot SHARED ${PSM_SRC_FILES})
target_link_libraries(PSM_robot  utilities lapack)

add_library(MTM_robot SHARED ${MTM_SRC_FILES})
target_link_libraries(MTM_robot  utilities lapack)





















