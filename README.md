# mapless_navigation

Ros: Melodic-desktop-full

## Summary:

I have created a simulation of a diff drive robot with a LIDAR sensor. A python code generates random landmarks in cube shapes. The distance between the landmarks are not less than 1m. I have added the error to the LIDAR as well as the Covariance. Used EKF package for sensor fusion and localisation. Plotted both odometry(yellow) only and the filtered odometry(red) to compare the effectiveness.

## Nodes list:
/ekf_localization_node
/gazebo
/gazebo_gui
/joint_state_publisher
/landmark_distance
/landmark_spawner
/move_base
/robot_state_publisher
/rosout

## To Run:

Build and source it
```
catkin_make
source devel/setup.bash
```

Start the gazebo
```
roslaunch model_description gazebo.launch
```
This file starts all the basic nodes for spawning the model and the landmarks.It also starts the node to calculate the distace of a landmark in x,y if its closer that 0.3m.To see that you can always echo the topic ```/landmark_distance```.
```
rostopic echo /landmark_distance
```

Start the rviz
```
roslaunch model_description rviz.launch
```

Start generating random goal points and send it to the navigation stack
```
rosrun robot_navigation move.py 
```


## Limitations:
The random goal point generator node currently generates points that are inside the lanmark region. It can be optimised in the future scope.