cmake_minimum_required(VERSION 3.10)
project(mrsim LANGUAGES CXX)

# --- Language / build type (colcon can override CMAKE_BUILD_TYPE) ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# --- Optimizations (kept from your original) ---
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
set(CMAKE_C_FLAGS_RELEASE     "${CMAKE_C_FLAGS_RELEASE} -O3 -DNDEBUG")

# --- ROS 2 / ament ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# --- OpenCV (system install) ---
find_package(OpenCV REQUIRED)

# --- Eigen: prefer package, fallback to standard include path ---
find_package(Eigen3 QUIET NO_MODULE)
if(Eigen3_FOUND)
  message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIRS}")
  set(EIGEN_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
else()
  message(STATUS "Eigen3 package not found; falling back to /usr/include/eigen3")
  set(EIGEN_INCLUDE_DIRS /usr/include/eigen3)
endif()

# --- JsonCpp: support both common imported targets ---
find_package(jsoncpp QUIET CONFIG)
if(TARGET jsoncpp_lib)
  set(JSONCPP_TARGET jsoncpp_lib)
elseif(TARGET JsonCpp::JsonCpp)
  set(JSONCPP_TARGET JsonCpp::JsonCpp)
else()
  message(FATAL_ERROR "jsoncpp not found: install libjsoncpp-dev (Ubuntu) or jsoncpp (brew).")
endif()

# --- Sources (everything you listed + the ROS bridge) ---
add_executable(simulator
  src/simulator.cpp 
  src/utils.cpp
  src/world.cpp
  src/freeflying.cpp
  src/lidar.cpp
  src/car.cpp
  src/arm.cpp
  src/object.cpp
  src/ros_bridge.cpp         # <— new file for ROS topics
)

# --- Includes ---
target_include_directories(simulator PRIVATE
  include                    # your package headers, e.g. include/mrsim/…
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

# --- Definitions (kept from your original) ---
target_compile_definitions(simulator PRIVATE UNIX)

# --- Link libs: keep your original + add ament deps ---
target_link_libraries(simulator
  ${OpenCV_LIBS}
  ${JSONCPP_TARGET}
)

ament_target_dependencies(simulator
  rclcpp geometry_msgs sensor_msgs std_msgs tf2 tf2_ros
)

# --- Install like a proper ROS 2 node ---
install(TARGETS simulator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
