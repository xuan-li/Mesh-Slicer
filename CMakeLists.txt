cmake_minimum_required(VERSION 3.0)

project(MeshSlicer)

set(CMAKE_CXX_STANDARD 14)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install CACHE PATH "cmake install prefix" FORCE)
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

if(MSVC)
  set(CMAKE_DEBUG_POSTFIX "d")
else()
  set(CMAKE_DEBUG_POSTFIX "")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake" CACHE STRING "Modules for CMake" FORCE)


####################### precompiled dependencies ####################

find_package(OpenMesh REQUIRED)
include_directories(src ${OpenMesh_INCLUDE_DIR})
link_libraries(${OpenMesh_LIBRARIES})

##################### import files ################

# if you want to import all:

file(GLOB SOURCES src/Mesh*.cpp src/D*.cpp)
file(GLOB HEADERS src/Mesh*.h src/D*.h)
add_library(${CMAKE_PROJECT_NAME} ${HEADERS} ${SOURCES})
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE ${OpenMesh_LIBRARIES})
add_definitions(-D_USE_MATH_DEFINES)
####################### add library #########################

# if you need dependencies:
# public mean the libraries are used in public APIs

#target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC ${OpenCV_LIBRARIES})
#target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})

######################## add executable ##########################

add_executable(${CMAKE_PROJECT_NAME}Test src/main.cpp)
target_link_libraries(${CMAKE_PROJECT_NAME}Test PRIVATE ${CMAKE_PROJECT_NAME})
add_definitions(-D_USE_MATH_DEFINES)

####################### installing configure ####################
configure_file(PackageConfig.cmake.in ${CMAKE_PROJECT_NAME}Config.cmake @ONLY)
install(TARGETS ${CMAKE_PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

