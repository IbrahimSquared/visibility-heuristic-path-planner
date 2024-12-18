# Visibility & Visibility Heuristic Path Planning

This repository contains an implementation of the algorithms provided in the paper: An Efficient Solution to the 2D Visibility Problem in Cartesian Grid Maps and its Application in Heuristic Path Planning ([ICRA2024](https://ieeexplore.ieee.org/document/10611529), [arxiv paper](https://arxiv.org/abs/2403.06494)). <br>

Provides a linear complexity & highly efficient 2D visibility solution, based on a solution to a linear first-order hyperbolic partial differential equation. The latter is the transport equation, as we define light/visibility as a transportable quantity, and transport it over the grid.

The algorithm functions in a dynamic-programming approach.

For example, a 101x101 grid can be processed at ~55kHz in C++ on an i9-13980HX processor. <br>
Code developed for C++20.

C++ code by default now computes standalone visibility for random environments and plots them using SFML using two methods: proposed and raycasting.
Benchmarks show that proposed method runs ~80 times faster than raycasting for an empty $1000\times{}1000$ gridmap. This speedup decreases for denser environments since raycasting terminates upon collisions. Using computeVisibilityUsingQueue() (has stopping criterion) can maintain speedups for denser environments. In all cases, proposed method remains faster.

## Visibility

### Two sample visibility polygons produced using b_visibility_shapes.m <br>

![alt text](https://github.com/IbrahimSquared/visibility-heuristic-path-planner/blob/main/Samples/visibility_polygon_5.jpg) <br>
![alt text](https://github.com/IbrahimSquared/visibility-heuristic-path-planner/blob/main/Samples/many_small_obstacles_3.jpg) <br>

### Binary visibility polygon <br>

![alt text](https://github.com/IbrahimSquared/visibility-heuristic-path-planner/blob/main/Samples/visibility_polygon_5_threshold.jpg) <br>

## Standalone Visibility Computation for a Sample Random Environment (SFML C++)

![alt text](https://github.com/IbrahimSquared/visibility-heuristic-path-planner/blob/main/Samples/SFMLstandAloneVisibility.png) <br>

## Standalone Visibility Computation for a Sample Random Environment Using Raycasting (SFML C++)

![alt text](https://github.com/IbrahimSquared/visibility-heuristic-path-planner/blob/main/Samples/SFMLrayCastingVisibility.png) <br>

## Benchmark Results Comparing Compute Time to Ray-Casting

![alt text](https://github.com/IbrahimSquared/visibility-heuristic-path-planner/blob/main/Samples/benchmarks.png) <br>
![alt text](https://github.com/IbrahimSquared/visibility-heuristic-path-planner/blob/main/Samples/benchmarks_speedup.png) <br>

## Visibility Heuristic Path Planner

### Sample solutions provided in the paper: <br>

![alt text](https://github.com/IbrahimSquared/visibility-heuristic-path-planner/blob/main/Samples/step_6.jpg) <br>
![alt text](https://github.com/IbrahimSquared/visibility-heuristic-path-planner/blob/main/Samples/maze_sol_0.png) <br>
![alt text](https://github.com/IbrahimSquared/visibility-heuristic-path-planner/blob/main/Samples/maze_sol_1.png) <br>
![alt text](https://github.com/IbrahimSquared/visibility-heuristic-path-planner/blob/main/Samples/lab_test_result.jpg) <br>
![alt text](https://github.com/IbrahimSquared/visibility-heuristic-path-planner/blob/main/Samples/maze_5_example.jpg) <br>
![alt text](https://github.com/IbrahimSquared/visibility-heuristic-path-planner/blob/main/Samples/maze_6_example.jpg) <br>

## Visibility Heuristic Path Planner SFML Generated Image Sample

![alt text](https://github.com/IbrahimSquared/visibility-heuristic-path-planner/blob/main/Samples/SFMLResultingPath.png) <br>

## MATLAB code

We provide a commented MATLAB implementation of all the demonstrations provided in the paper. <br>

1. The environments can be randomized by uncommeting the loaded seed, but we provide the seeds that we used for reproducibility purposes (rnd_1.mat rnd_2.mat).
2. a_quiver_plots.m plots the components a(x,y) and b(x,y) that govern the solution behaviour of the partial differential equation (lines-of-sight vs curves-of-sight are illustrated in the paper).
3. b_visibility_shapes.m plots the visibility polygon provided in the paper (environment can be randomized also).
4. c_sample_planner_solving_random_environments.m solves a path planning problem using the visibility heuristic path planner solution and plots the results.
5. d_sample_planner_solving_maze same as 3- but loads maze images and solves them.
6. e_used_for_video_submission.m self-explanatory.
7. f_comparison_to_a_star.m used to get the comparison results against astar.
8. test_environment_generator.m standalone, tests generating random environments with given settings making sure that start and end positions are not inside obstacles

## C++ code

We provide a C++ implementation that is interfaced with MATLAB too. <br>
How to use the C++ code: <br>
The C++ code has a parser that parses settings from settings.config file, and can work in two modes. In the first mode, a random environment with a given grid size is generated containing a specific number of obstacles that have given dimensions (all loaded via settings.config). The start and end points can also be specified as well as some other settings. <br>
In the second mode, the C++ code uses SFML to load an image representing the environment (mazes in this case). The directory of the image is set in the settings.config also as well as start and end point positions.
In both modes, the solver outputs .txt files which are loaded by MATLAB and plotted as meshes for viewing purposes. The MATLAB interface uses the solution lightSource_enum to also plots lines to visualize the path. lightSource_enum essentially stores the parent of every explored point in the grid, meaning we can retract the solution with it.

Important notes before using: <br>

1. In the load image mode, the dimensions of the image may be flipped so double check when setting start and end positions.
2. For both modes, make sure to set equal start and end points in both the MATLAB interface and in settings.config. In MATLAB, the values have to be incremented by 1 to match the C++ code.
3. Instead, you can try to parse settings.config also in MATLAB.
4. Set the correct image path in settings.config.

To interface with MATLAB, the code interface.m calls visibility_heuristic_planner.exe using visibility_heuristic_planner.bat, where the components of the .bat file are: <br>
`set path=%path:C:\Program Files\MATLAB\R2022b\bin\win64;=%` <br>
`visibility_heuristic_planner.exe` <br>
Make sure to change the path for your MATLAB installation directory inside the .bat (and use the proper version). <br>
The code then reads the results and plots them nicely.

## Important: Standard for inputting/reading images in C++

Mode=2 reads an image map, the path of which is specified in imagePath in settings.config, for example lab_image_edited.png (893x646) in the folder images. <br>
Bottom left corner is the origin. <br>
This is relevant for selecting start/end points. This standard can ofcourse be changed.

## To build or compile using cmake in Linux

Required: <br>
cmake and g++: <br>
`sudo apt install cmake` <br>
`sudo apt install g++` <br>
libsfml-dev: <br>
`sudo apt-get install libsfml-dev` <br>
Set compiler path (or comment that part), make sure SFML libraries are installed, then: <br>
`mkdir build && cd build` <br>
`cmake ..` <br>
`make`

## Instructions to build the C++ code on Windows in Visual Studio Code

We provide tasks.json, c_cpp_properties.json, and launch.json for building and launching the code in Visual Studio Code. <br>
Make sure to change the compiler path in tasks.json for both debug and release modes: <br>
`"command": "C:\\Workdir\\Programs\\msys64\\mingw64\\bin\\g++.exe"`
The tasks.json automatically links the SFML libraries with the argument `"-lsfml-graphics"`. <br>
For debug purposes, change "miDebuggerPath": "C:\\Workdir\\Programs\\msys64\\mingw64\\bin\\gdb.exe" path as well in launch.json. <br>

Dependencies: <br>
If you are using Visual Studio Code and MSYS2: <br>
Install SFML in MSYS2 using: <br>
`pacman -S mingw-w64-x86_64-sfml` <br>

## Instructions to build the C++ code on Windows in Visual Studio Code using CMakeLists.txt

We provide CMakeLists.txt for easy building and compilation too. Install CMake Tools extension on VSCode and configure the kit and the generator. <br>
We used: <br>
`pacman -S mingw-w64-x86_64-cmake` <br>
Make sure cmake is working using `cmake --version`, a reload may be necessary <br>
Set the cmakePath accordingly in settings.json: <br>
`"cmake.cmakePath": "C:\\Workdir\\Programs\\msys64\\mingw64\\bin\\cmake.exe",`
