# HPSP
Core modules of **h**omotopic **p**ath **s**et **p**lanning (HPSP), especially the module of passage-aware optimal path planning based on sampling-based planners. It is in active development.
## Requirements
We use ROS to organize the project, but the code should also be compilable independently. Other libraries include OpenCV for visualization and Eigen for computation.
## Minimal example
Go to the folder where the code is cloned. Build the project as
```sh
catkin build
```
Source the project as
```sh
source devel/setup.bash
```
Then, examples defined in the CMakeLists.text is runable, e.g.,
```sh
rosrun path_set_planning planning_cost_comparison
```
The source code file name of executables can be found in the CMakeLists.text. Changes can be made to source code for different tests.
## TODO
