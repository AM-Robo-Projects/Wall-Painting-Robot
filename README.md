# Wall-Painting-Robot
A wall painting UR5e using Livox lidar for mapping walls, segmenting it and then move the robot along it for painting. 

## Launching the Simulation

```bash
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
```

## Launching the Point Cloud Processing Node 

```bash
ros2 run ur_simulation_gazebo clustering
```

## Launching Dynamic Reconfigure 

The dynamic reconfig is to adjust the cropping distance between the lidar and the walls to crop the desired point clouds only with no need to process the whole point cloud. 

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

## Important ROS topics 

/detected_walls -> gives the corners of the wall x,y,z in the format  

[lower_corner_left (x,y,z), upper_corner_left(x,y,z), upper_corner_right(x,y,z), lower_corner_right(x,y,z) ]

