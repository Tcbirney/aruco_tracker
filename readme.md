### Description

This project demonstrates a mapping, navigation, and aruco recognition based visual servoing scenario for the turtlebot platform in multiple worlds

### Setup

__THIS WILL INSTALL COMPONENTS TO YOUR SYSTEM__

In order to enable the catkin_ws, cd into the project dir and run

```
chmod 777 init_proj.sh

./init_proj.sh
```

Copy the contents of the gazebo_models_perception_group_4 to be in your .gazebo folder in your home directory

The URDF for the robot is unused but available in this repo.

### Run Instructions

To Launch The Default World And Rvis

`roslaunch PerceptionProject turtlebot3_world_with_camera.launch`

To Launch Mapping 

`roslaunch PerceptionProject map_gen.launch`

To Launch Navigation
```
roslaunch PerceptionProject turtlebot3_world_with_camera.launch
roslaunch PerceptionProject nav_stack.launch
rosrun PerceptionProject explore_then_dock.py
```

__If you want to use the second map then use__


To Launch The Default World And Rvis

`roslaunch PerceptionProject turtlebot3_world_with_camera.launch x_pos:=0.5 y_pos:=1 world_name:=three_aruco_world.world`

To Launch Mapping 

`roslaunch PerceptionProject map_gen.launch map:=3map.yaml`

To Launch Navigation
```
roslaunch PerceptionProject turtlebot3_world_with_camera.launch x_pos:=0.5 y_pos:=1 world_name:=three_aruco_world.world
roslaunch PerceptionProject nav_stack.launch
rosrun PerceptionProject explore_then_dock.py
```

When you launch the nav stack you will need to click the green arrow to set the initial pose location of the robot in rviz

You will need to go in to the explore_then_dock.py file and change the pose set to the second list for the second map because that was not externally parameterized

### Hardware Instructions:

To run the hardware code, refer to the README.md in the `robot_scripts` directory
