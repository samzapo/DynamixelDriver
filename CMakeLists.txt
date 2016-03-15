cmake_minimum_required(VERSION 2.8.3)

project(dxl)

option(BUILD_SHARED_LIBS "Build as a shared library?" ON)
#SET(CMAKE_MODULE_PATH "./CMakeModules")

INCLUDE_DIRECTORIES(
  include
)

link_directories(/usr/local/lib /usr/lib .)

# build libraries
ADD_LIBRARY(dxl
 Dynamixel.cpp
 DynamixelComm.cpp
)

target_link_libraries(
  dxl
)

# fix the C++ linking error on 64-bit Linux
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

set(CMAKE_INSTALL_PREFIX /usr/local)
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set_target_properties(dxl PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS dxl DESTINATION lib)
install (DIRECTORY ${CMAKE_SOURCE_DIR}/include/dxl DESTINATION include)

# build test
ADD_EXECUTABLE(dxl_test
 Dynamixel.cpp
 DynamixelComm.cpp
)
target_link_libraries(
  dxl_test
)

