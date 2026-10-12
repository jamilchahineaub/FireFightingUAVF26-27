cmake_minimum_required(VERSION 3.16)
project(cl415_gz_plugins)

# gz harmonic from the ros jazzy vendor packages (source /opt/ros/jazzy/setup.bash first)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-common5 REQUIRED)
# VRX wavefield
set(VRX_ROOT "$ENV{HOME}/vrx_ws")
set(VRX_WAVE_INCLUDE "${VRX_ROOT}/src/vrx/vrx_gz/src")

find_library(VRX_WAVES_LIB
  NAMES Waves
  PATHS "${VRX_ROOT}/install/lib"
  NO_DEFAULT_PATH
)

if(NOT VRX_WAVES_LIB)
  message(FATAL_ERROR "VRX libWaves.so not found")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(Cl415PropellerMap SHARED PropellerMap.cc)
target_link_libraries(Cl415PropellerMap PRIVATE
  gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 gz-common5::gz-common5)

add_library(Cl415Magnetometer SHARED Magnetometer.cc)
target_link_libraries(Cl415Magnetometer PRIVATE
  gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::gz-transport13 gz-msgs10::gz-msgs10 gz-common5::gz-common5)

add_library(Cl415FlyingBoatHull SHARED FlyingBoatHull.cc)

target_include_directories(Cl415FlyingBoatHull PRIVATE
  "${VRX_WAVE_INCLUDE}"
)

target_link_libraries(Cl415FlyingBoatHull PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-common5::gz-common5
  "${VRX_WAVES_LIB}"
)

set_target_properties(Cl415FlyingBoatHull PROPERTIES
  BUILD_RPATH "${VRX_ROOT}/install/lib"
)

add_library(Cl415ChaseCamera SHARED ChaseCamera.cc)
target_link_libraries(Cl415ChaseCamera PRIVATE
  gz-sim8::gz-sim8 gz-plugin2::register gz-common5::gz-common5)
