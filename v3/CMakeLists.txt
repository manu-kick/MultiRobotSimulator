# set(CMAKE_CXX_STANDARD 17)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(simulator)


# Set the output directory for the build executables and libraries
ADD_DEFINITIONS(-DUNIX)
MESSAGE(STATUS "Compiling on Unix")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG") 
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -DNDEBUG") 
SET(CMAKE_BUILD_TYPE Release)

# Find Eigen3
# FIND_PACKAGE(Eigen3 REQUIRED)
# INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(/usr/include/eigen3)

# Find Opencv
set(OpenCV_DIR "/opt/homebrew/opt/opencv@4/share/opencv4")
FIND_PACKAGE(OpenCV  REQUIRED)
# JsonCpp from Homebrew
find_package(JsonCpp REQUIRED)    
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})


add_executable(simulator
  src/simulator.cpp 
  src/world.cpp
  src/freeflying.cpp
  src/lidar.cpp
  src/car.cpp
  src/arm.cpp
  src/object.cpp

#   src/test_manu.cpp
  )

target_link_libraries(
  simulator 
  ${OpenCV_LIBS} 
  JsonCpp::JsonCpp
)

