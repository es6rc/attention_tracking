# CMakeLists files in this project can
# refer to the root source directory of the project as ${HELLO_SOURCE_DIR} and
# to the root binary directory of the project as ${HELLO_BINARY_DIR}.
cmake_minimum_required (VERSION 2.8.11)
project (at)

set ( CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/bin")

# Local libraries
INCLUDE_DIRECTORIES (${CMAKE_SOURCE_DIR}/include)
# Local sources
SET (SOURCES
     src/serial.cpp
     src/sprotocol.cpp )

# Append necessary module path 
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/modules")
# Find TBB
find_package(TBB REQUIRED)

### Dependent Libs
# Add OpenCV 3.4.0
find_package(OpenCV 3.4.0 REQUIRED COMPONENTS)
INCLUDE_DIRECTORIES (${OpenCV_INCLUDE_DIRS})

# Add OpenFace packages
find_package(OpenFace REQUIRED )
#find_package(OpenFace REQUIRED PATHS $(VAR) NO_DEFAULT_PATH)
message("OpenFace_INCLUDE_DIRS = ${OpenFace_INCLUDE_DIRS}")
INCLUDE_DIRECTORIES(${OpenFace_INCLUDE_DIRS})

# Add Eigen3 Directories
SET( EIGEN3_INCLUDE_DIR "/usr/include/eigen3" )
IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

# Add catkin and ROS related packages
#find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg geometry_msgs)
#INCLUDE_DIRECTORIES (include ${catkin_INCLUDE_DIRS})
#message("catkin_INCLUDE_DIRS = ${catkin_INCLUDE_DIRS}")

#include_directories(/home/z/CppDependencies/OpenFace/install/include/OpenFace)
add_executable (at main.cpp ${SOURCES})
add_executable (readservo mainlog.cpp ${SOURCES})
target_link_libraries(readservo ${OpenFace_LIBRARIES})
target_link_libraries(at ${OpenFace_LIBRARIES})
#target_link_libraries(at ${catkin_LIBRARIES})
target_link_libraries(at ${OpenCV_LIBS} )

# Set install prefix
message ("CMAKE_BINARY_DIR =  ${CMAKE_BINARY_DIR}")
message ("CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}" )
install (TARGETS at DESTINATION ${CMAKE_BINARY_DIR}/bin)