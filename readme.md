## Build procedures:
1. Clone turtlebot_walker branch from github by inputting
```
git clone -b turtlebot_walker https://github.com/yhsueh/beginner_tutorials.git
```

2. Move beginner_tutorials into src folder in your catkin_ws.

2. Make sure ROS_PACKAGE_PATH enviroment variable contain the workspace folder. This is done by sourcing the generated setup file under devel folder in the parent directory.

3. Input 
```
catkin_make
```
to build the ROS package.

## Procedures for running the turtlebot simulation:
1. Run roslaunch file to start gazebo, and two necessary nodes.
```
roslaunch turtlebot_walker turtlebot_world.launch
```
The simulation can be recorded if record_flag is set to 1;
```
roslaunch turtlebot_walker turtlebot_world.launch record_flag:=1
```

2. Switch back to gazebo and view the simulation.

3. See what topics are recorded with rosbag info and rosbag play. Change directory to results folder. Before running rosbag play, make sure roscore is up and running.
```
roscore
cd {"turtlebot_walker"}/results
rosbag info bagfile.bag
rosbag play bagfile.bag
```