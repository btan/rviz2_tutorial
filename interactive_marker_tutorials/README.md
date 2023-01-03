# Rviz2 Tutorial Migration from Rviz1
## Interactive Markers (Basic Control) in package: interactive_marker_tutorials

The code is modified from the link: https://github.com/ros-visualization/visualization_tutorials/blob/ros2/interactive_marker_tutorials/src/basic_controls.cpp.

## Create a package
```sh
ros2 pkg create --build-type ament_cmake --node-name basic_controls interactive_marker_tutorials
```
## Build a package
```sh
colcon build --packages-select interactive_marker_tutorials
```

## Source the setup file and run the package
```sh
. install/setup.bash
ros2 run interactive_marker_tutorials basic_controls
```

## View Marker
Launch another terminal
```sh
ros2 run rviz2 rviz2
```
Set the Fixed Frame field to "/base_linke", and add Interactive Markers display
[RVIZ screenshot](https://github.com/btan/rviz2_tutorial/tree/main/interactive_marker_tutorials/img/basicControl.png?raw=true)
