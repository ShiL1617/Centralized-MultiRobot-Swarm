# WINTER 2017 PROJECT

## MULTI-ROBOT FORMATION CONTROL

After building the package by going to the catkin_ws/ directory then running

```
catkin_make
```
please source the appropriate .sh files:


```
source /opt/ros/indigo/setup.bash
source devel/setup.bash
```
Run the following two launch files to show demos of

trajectory following:

```
roslaunch winter_project trajectoryfollowing_combo.launch 
```

shape formation (RViz unfortunately crashes a good amount with this one...):

```
roslaunch winter_project shape_formation_square_spiral.launch 
```
