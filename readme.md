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

## Procedures for viewing tf:
1. Create four terminals. Make sure to source the package in each of the terminal.

2. Run roslaunch file to start gazebo.
```
roslaunch turtlebot_walker turtlebot_world.launch
```


## Procedures for using roslaunch to record topics and read the bagfile:
1. Similar to last section. Create four terminals and repeat the procedures until the forth step.

2. Input
```
roslaunch beginner_tutorials node.launch record_flag:=1
```
If no argument is passed, record_flag is set to 0 by default, and no topics is recorded.

3. 
See what topics is recorded
```
cd {"beginner_package"}/results
rosbag info bagfile.bag
```

4. Replay chatter messages using listener node
```
rosrun beginner_tutorials listener

```
then in a different terminal
```
rosbag play bagfile.bag
```


