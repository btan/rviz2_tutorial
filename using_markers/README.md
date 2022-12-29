# Rviz2 Tutorial Migration from Rviz1
## Markers: Basic Shapes in package: using_markers

The code is taken from the link: https://raw.githubusercontent.com/ros-visualization/visualization_tutorials/indigo-devel/visualization_marker_tutorials/src/basic_shapes.cpp.

## Create a package
```sh
ros2 pkg create --build-type ament_cmake --node-name basic_shapes using_markers
```
## Build a package
```sh
colcon build --packages-select using_markers
```

## Source the setup file and run the package
```sh
source install/setup.bash
ros2 run using_markers basic_shapes
```

## View Marker
Launch another terminal
```sh
ros2 run rviz2 rviz2
```
Set the Fixed Frame field to "/my_frame", and add a Markers display
[RVIZ screenshot](https://github.com/btan/rviz2_tutorial/tree/main/using_markers/img/rviz2_usingmarkers.png?raw=true)


## Markers: Points and Lines
## Build a package
```sh
colcon build --packages-select using_markers
```
## Source the setup file and run the package
```sh
source install/setup.bash
ros2 run using_markers points_and_lines
```

## View Marker
Launch another terminal
```sh
ros2 run rviz2 rviz2
```
Set the Fixed Frame field to "/my_frame", and add a Markers display