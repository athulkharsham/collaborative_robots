# collaborative_robots
Manipulator with pick and place and differential drive robot to carry goods

<!-- USAGE -->

## Usage
1. Start the robot stack with moveit
```sh
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
```
2. Start pick and place
```sh
ros2 launch moveit2_scripts cartesian_path.launch.py
```