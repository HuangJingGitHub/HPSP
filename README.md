# HPSP
Core modules of **h**omotopic **p**ath **s**et **p**lanning (HPSP), especially the module of passage-aware optimal path planning based on sampling-based planners. It is in active development.
## Requirements
We use ROS to organize the project, but the code should also be compilable independently. Other libraries include OpenCV for visualization and Eigen for computation.
## Minimal example
First, run `roscore` in a separate terminal. Then, go to the folder where the code is cloned. Build the project as
```sh
catkin build
```
Source the project as
```sh
source develop/setup.bash
```
Then, we can run the examples defined in the Cmake.text, e.g.,
```sh
rosrun path_set_planning x
```
