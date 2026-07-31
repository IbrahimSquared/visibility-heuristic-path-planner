cmake_minimum_required(VERSION 3.10)

project(visibility_heuristic_planner)

# Set the compiler to use if needed 
# set(CMAKE_CXX_COMPILER C:/Workdir/Programs/msys64/mingw64/bin/g++.exe)
# set(CMAKE_CXX_COMPILER /usr/bin/g++)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -flto -Wall -Wpedantic -Wextra -Wnull-dereference")
set(CMAKE_EXPORT_COMPILE_COMMANDS on)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

# Find dependencies.
find_package(SFML 2.5 COMPONENTS graphics REQUIRED)
option(VHPP_ENABLE_OPENMP "Enable OpenMP acceleration" ON)
if(VHPP_ENABLE_OPENMP)
  find_package(OpenMP)
endif()

add_library(vhpp_core
  src/environment.cpp
  src/visibilityBasedSolver.cpp
  src/parser.cpp
)
target_include_directories(vhpp_core PUBLIC include)
target_link_libraries(vhpp_core PUBLIC sfml-graphics)

if(VHPP_ENABLE_OPENMP AND OpenMP_CXX_FOUND)
  target_link_libraries(vhpp_core PUBLIC OpenMP::OpenMP_CXX)
  message(STATUS "OpenMP acceleration enabled")
else()
  message(STATUS "OpenMP unavailable or disabled; using the serial fallback")
endif()

add_executable(visibility_heuristic_planner src/main.cpp)
target_link_libraries(visibility_heuristic_planner PRIVATE vhpp_core)

include(CTest)
if(BUILD_TESTING)
  add_executable(vhpp_solver_regression tests/solver_regression.cpp)
  target_link_libraries(vhpp_solver_regression PRIVATE vhpp_core)
  set_target_properties(vhpp_solver_regression PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )
  add_test(NAME solver_regression COMMAND vhpp_solver_regression)
endif()
