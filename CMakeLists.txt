cmake_minimum_required(VERSION 2.8.3)

project(dxl)

option(BUILD_SHARED_LIBS "Build as a shared library?" ON)
#SET(CMAKE_MODULE_PATH "./CMakeModules")

INCLUDE_DIRECTORIES(
  include
)

link_directories(.)

# build libraries
ADD_LIBRARY(dxl_comm
 DynamixelComm.cpp
)
ADD_LIBRARY(dxl
 DynamixelComm.cpp
)

target_link_libraries(
  dxl_comm
)
target_link_libraries(
  dxl dxl_comm
)

set(CMAKE_INSTALL_PREFIX /usr/local)
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
install (TARGETS dxl DESTINATION lib)
install (DIRECTORY ${CMAKE_SOURCE_DIR}/include/dxl DESTINATION include)

# build test
ADD_EXECUTABLE(dxl_test
  TestDynamixel.cpp
)

target_link_libraries(
  dxl_test dxl Ravelin
)

