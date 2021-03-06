project(QCIUtils CXX)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_DISABLE_IN_SOURCE_BUILDS ON)
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
message(STATUS "C++ version ${CXX_STANDARD} configured.")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -Wno-long-long -pedantic -Wno-unused-function")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules")

if (EXISTS ${CMAKE_SOURCE_DIR}/tpls/fire)
  message(STATUS "Fire found in tpls")
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/tpls/fire/cmake/Modules")
  include(tests)
endif() 

find_package(MPI REQUIRED)

if(NOT Boost_FOUND)
  find_package(Boost COMPONENTS system REQUIRED)
  message(STATUS "Found Boost Headers = ${Boost_INCLUDE_DIRS}")
  message(STATUS "Found Boost Libraries = ${Boost_LIBRARIES}")
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(${MPI_INCLUDE_PATH})
include_directories(${Boost_INCLUDE_DIRS})
include_directories("${CMAKE_SOURCE_DIR}/factory")
include_directories("${CMAKE_SOURCE_DIR}/graph")
include_directories("${CMAKE_SOURCE_DIR}/exception")

set (PACKAGE_NAME "QCI-Utils")
set (PACKAGE_DESCRIPTION "ORNL QCI Common Utilities")

file (GLOB_RECURSE HEADERS *.hpp)
file (GLOB test_files factory/tests/*.cpp graph/tests/*.cpp)


add_tests("${test_files}" "${CMAKE_CURRENT_SOURCE_DIR}" "")
  
install (FILES ${HEADERS} DESTINATION include)
