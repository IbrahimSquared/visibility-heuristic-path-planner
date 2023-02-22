# Visibility & Visibility Heuristic Path Planning
This repository contains an implementation of the algorithms provided in the paper: to-be-added.

# Visibility
Two sample visibility polygons produced using b_visibility_shapes.m <br>
![alt text](https://github.com/IbrahimSquared1/visibility-heuristic-path-planner/blob/main/Samples/visibility_polygon_5.jpg) <br>
![alt text](https://github.com/IbrahimSquared1/visibility-heuristic-path-planner/blob/main/Samples/many_small_obstacles_3.jpg) <br>

Thresholded visibility polygon <br>
![alt text](https://github.com/IbrahimSquared1/visibility-heuristic-path-planner/blob/main/Samples/visibility_polygon_5_threshold.jpg) <br>

# Visibility Heuristic Path Planner
Sample solutions provided in the paper: <br>
![alt text](https://github.com/IbrahimSquared1/visibility-heuristic-path-planner/blob/main/Samples/step_6.jpg) <br>
![alt text](https://github.com/IbrahimSquared1/visibility-heuristic-path-planner/blob/main/Samples/maze_sol_0.png) <br>
![alt text](https://github.com/IbrahimSquared1/visibility-heuristic-path-planner/blob/main/Samples/maze_sol_1.png) <br>

# MATLAB code
We provide a commented MATLAB implementation of all the demonstrations provided in the paper.
  0. The environments can be randomized by uncommeting the loaded seed, but we provide the seeds that we used for reproducibility purposes (rnd_1.mat rnd_2.mat).
  1. a_quiver_plots.m plots the components a(x,y) and b(x,y) that govern the solution behaviour of the partial differential equation (lines-of-sight vs curves-of-sight are illustrated in the paper).
  2. b_visibility_shapes.m plots the visibility polygon provided in the paper (environment can be randomized also).
  3. c_sample_planner_solving_random_environments.m solves a path planning problem using the visibility heuristic path planner solution and plots the results.
  4. d_sample_planner_solving_maze same as 3- but loads maze images and solves them.
  5. e_used_for_video_submission.m self-explanatory.
  6. f_comparison_to_a_star.m used to get the comparison results against astar.
  7. test_environment_generator.m standalone, tests generating random environments with given settings making sure that start and end positions are not inside obstacles

# C++ code
We provide a C++ implementation that is interfaced with MATLAB too. <br>
How to use the C++ code: <br>
The C++ code has a parser that parses settings from settings.config file, and can work in two modes. In the first mode, a random environment with a given grid size is generated containing a specific number of obstacles that have given dimensions (all loaded via settings.config). The start and end points can also be specified as well as some other settings. <br>
In the second mode, the C++ code uses SFML to load an image representing the environment (mazes in this case). The directory of the image is set in the settings.config also as well as start and end point positions.
In both modes, the solver outputs .txt files which are loaded by MATLAB and plotted as meshes for viewing purposes.
We provide main.exe which is a built executable of the code. <br>
Important note: the dimensions of the image may be flipped so double check when setting start and end position for the mode with loading images. <br>

To interface with MATLAB, the code read_serialized_heuristic.m calls main.exe using main.bat, where the components of the .bat file are: <br>
set path=%path:C:\Program Files\MATLAB\R2022b\bin;=% <br>
main.exe <br>
Make sure to change the path for your MATLAB installation directory inside the .bat (and use the proper version). <br>
The code then reads the results and plots them nicely.

# Instructions to build the C++ code on Windows in Visual Studio Code
We provide tasks.json, c_cpp_properties.json, and launch.json for building and launching the code in Visual Studio Code. <br>
Make sure to change the compiler path in tasks.json for both debug and release modes: <br>
"command": "C:\\Workdir\\Programs\\msys64\\mingw64\\bin\\g++.exe"
The tasks.json automatically link the SFML libraries with the argument "-lsfml-graphics". <br>
For debug purposes, change "miDebuggerPath": "C:\\Workdir\\Programs\\msys64\\mingw64\\bin\\gdb.exe" path as well in launch.json. <br>

Dependencies: <br>
If you are using Visual Studio Code and MSYS2: <br>
Install SFML in MSYS2 using:  <br>
pacman -S mingw-w64-x86_64-sfml <br>
 <br>