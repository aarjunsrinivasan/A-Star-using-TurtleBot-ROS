# Implementation of A* algorithm on TurtleBot-ROS Gazebo Simulation



## Overview - Implemntation for A* Path Planning algorithm
In this project, A* algorithm is simulated on Turtle bot differential drive (non-holonomic) mobile robot in a defined static world. We are using Turtlebot in ROS-Gazebo for the simulation. The code plans the path for Turtlebot to follow which  contains linear and angular velocities needed by the robot to reach its goal position. A ROS node publishes these velocities at regular 
intervals to simulate Turtlebot's movement in Gazebo. The map in which the bot navigates is shown below.

<p align="center">
  <p align="center"><img src="astar.png"></p>
  <br><b>Figure 1 - Turtlebot-3 Burger moving in known world using velocities provided by A* planner</b><br>
</p>

A* Path planning algorithm is very effective for known environment. 

The algorithm is similar to Djikstra's algorithm except for the fact that the total cost is a combination of cost/distance of the current node to the start node (Path cost) and current node to the goal node (heuristic cost). TurtleBot is a differential drive robot and In this implementation I am assuming the robot can travel in one of the 8 rpm left and rpm right combinations. The path cost of traveling to an adjacent node in the above mentioned direction is set to the distance travelled. The Heuristic cost is calculated by the straight line distance between the current node and the goal node. 

The total cost is calculated as a combination of path cost + heusristic cost.
Once the user inputs the start and end nodes the software calculates the shortest path between the two nodes. The below picture shows the visual of the selected path. 


## Authors

- [Arjun Srinivasan](https://github.com/aarjunsrinivasan)


## Dependencies

- Ubuntu 16.04/18.04
- ROS Kinetic
- Gazebo
- Turtlebot3 Packages
- Python Packages: Numpy, OpenCV-Python, Math, Queue
- Install Turtlebot-3 package and its dependencies:

In your .bashrc file, include the following statements and source it.

source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger

To create a catkin workspace and install dependecies for turtlebot3 package, run the following commands:

$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ cd ~/catkin_ws/
$ catkin_make


From the downloaded file,which is the package "searchbot" folder, copy to /catkin_ws/src/ and run the following commands:

$ cd ~/catkin_ws/
$ source ~/catkin_ws/devel/setup.bash

## Instructions to run:

The inputs from the user are coordinates of start point, orientation of start point (in degrees), coordinates of goal point, two RPM values and clearance.

The orientation of goal point is taken by default as 0.

The clearance given by the user is 0.1 (minimum clearance).

In the terminal run the following command:

roslaunch searchbot astar.launch  x_pos:=4.5 y_pos:=3.0 yaw:=3.142

The x,y coordinates of start point in gazebo should be given in arguments 'x_pos' and 'y_pos' respectively in the terminal. The argument 'yaw' is (3.14 + orientation at start of the robot in radians). This ensures the turtlebot to spawn at correct position and orientation in gazebo.

The start coordinates and goal coordinates should be given as negative of the coordinates as observed in right-handed system because the map given has negative axes. For example, if the robot start point is bottom left (-4.5,-4.5) in right-handed coordinate system, it should be given as (4.5,4.5).

Inputs for the submitted videos:

Video 1:
For Manual input enter 1 else enter 2
1
Enter the clearance.1
Please enter the rpm1:3
Please enter the rpm2:4
Please enter start point x coordinate:4.5
Please enter start point y coordinate:3.0
Please enter start orientation in degrees:0
Please enter goal point x coordinate:-4.5
Please enter goal point y coordinate:-3.0

Video2:
For Manual input enter 1 else enter 2
1
Enter the clearance.1
Please enter the rpm1:3
Please enter the rpm2:4
Please enter start point x coordinate:4.5
Please enter start point y coordinate:3.0
Please enter start orientation in degrees:0
Please enter goal point x coordinate:0
Please enter goal point y coordinate:3.0


