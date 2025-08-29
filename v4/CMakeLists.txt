cmake_minimum_required(VERSION 3.10)
project(simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

add_definitions(-DUNIX)
message(STATUS "Compiling on Unix")

# Optimizations
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
set(CMAKE_C_FLAGS_RELEASE     "${CMAKE_C_FLAGS_RELEASE} -O3 -DNDEBUG")

# Eigen (system install)
include_directories(/usr/include/eigen3)

# OpenCV (system install; do NOT hardcode a mac path here)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# JsonCpp (Ubuntu provides imported target 'jsoncpp_lib'; some platforms provide 'JsonCpp::JsonCpp')
find_package(jsoncpp QUIET CONFIG)
if(TARGET jsoncpp_lib)
  set(JSONCPP_TARGET jsoncpp_lib)
elseif(TARGET JsonCpp::JsonCpp)
  set(JSONCPP_TARGET JsonCpp::JsonCpp)
else()
  message(FATAL_ERROR "jsoncpp not found: install libjsoncpp-dev (Ubuntu) or jsoncpp (brew).")
endif()

add_executable(simulator
  src/simulator.cpp 
  src/world.cpp
  src/freeflying.cpp
  src/lidar.cpp
  src/car.cpp
  src/arm.cpp
  src/object.cpp
)

target_link_libraries(simulator PRIVATE ${OpenCV_LIBS} ${JSONCPP_TARGET})
