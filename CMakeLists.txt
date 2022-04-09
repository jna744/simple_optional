cmake_minimum_required(VERSION 3.13)

project(simple_optional VERSION 0.1)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  set(SO_PRIMARY_PROJECT true)
  set(CMAKE_EXPORT_COMPILE_COMMANDS true)
endif()

add_library(${PROJECT_NAME} INTERFACE)
add_library(simple::optional ALIAS ${PROJECT_NAME})

target_include_directories(
  ${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING AND SO_PRIMARY_PROJECT)
  add_subdirectory(test)
endif()
