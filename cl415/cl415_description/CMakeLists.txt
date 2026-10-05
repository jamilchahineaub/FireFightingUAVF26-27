cmake_minimum_required(VERSION 3.8)
project(cl415_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch worlds config models scripts
  DESTINATION share/${PROJECT_NAME})

# puts <install>/share on GZ_SIM_RESOURCE_PATH so package:// (turned into model://) mesh uris resolve
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
