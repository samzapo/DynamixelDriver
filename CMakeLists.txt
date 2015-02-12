cmake_minimum_required(VERSION 2.8.3)

project(dxl)

option(BUILD_SHARED_LIBS "Build as a shared library?" ON)
#SET(CMAKE_MODULE_PATH "./CMakeModules")

INCLUDE_DIRECTORIES(
  include
)

link_directories()

ADD_EXECUTABLE(${PROJECT_NAME}_test
  DynamixelComm.cpp
  Dynamixel.cpp
)
ADD_LIBRARY(${PROJECT_NAME}
 DynamixelComm.cpp
 Dynamixel.cpp
  )

target_link_libraries(
  ${PROJECT_NAME} Ravelin
)
target_link_libraries(
  ${PROJECT_NAME}_test Ravelin
)

set(CMAKE_INSTALL_PREFIX /usr/local)
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
install (TARGETS ${PROJECT_NAME} DESTINATION lib)
install (DIRECTORY ${CMAKE_SOURCE_DIR}/include/dxl DESTINATION include)
