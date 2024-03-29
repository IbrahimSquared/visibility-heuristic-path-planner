cmake_minimum_required(VERSION 3.10)

project(visibility_heuristic_planner)

# Set the compiler to use if needed 
# set(CMAKE_CXX_COMPILER C:/Workdir/Programs/msys64/mingw64/bin/g++.exe)
# set(CMAKE_CXX_COMPILER /usr/bin/g++)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Ofast -march=native -flto")
set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

include_directories(include)
# include_directories(C:/Workdir/Programs/msys64/mingw64/include) if needed
add_executable(visibility_heuristic_planner src/main.cpp src/environment.cpp src/visibilityBasedSolver.cpp src/parser.cpp)

# Find the SFML package
find_package(SFML 2.5 COMPONENTS graphics REQUIRED)
# Link the SFML graphics library
target_link_libraries(visibility_heuristic_planner PRIVATE sfml-graphics)
