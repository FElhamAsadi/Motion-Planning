# Path Plannning Project
Self-Driving Engineering - Udacity Nanodegree
## Overview
The goal of this project is to safely navigate around a virtual highway (on a simulator provided by Udacity - https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization, sensor fusion data and a sparse map list of waypoints around the highway are provided to the path-planner module. The ultimate goal of this module is designing a trajectory which allows the car to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. It should avoid collision with other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it will take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

---
## Files
The project consists of the following files:
- [main.cpp ](../src/main.cpp): The main c++ file that implements the communication with the simulator.
- [path_planner.cpp](../src/path_planner.cpp): The behavioural planning and trajectory design are done in this file.
- [vehicle.cpp](../src/vehicle.cpp): This define a class which holds the localization and senors' data about the ego car and its surronding vehicles. 
- [spline.h](../src/spline.h): This is used for generating smooth trajectories.
- [helpers.cpp](../src/helpers.cpp): Utility functions are hold here.
- README.md

---
## Data
#### Map
 The map of the highway is in data/highway_map.txt. Each waypoint in the list contains  [x,y,s,dx,dy] values where x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop. The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554 meter.

#### Ego car's localization Data (No Noise)
["x"] The car's x position in map coordinates
["y"] The car's y position in map coordinates
["s"] The car's s position in frenet coordinates
["d"] The car's d position in frenet coordinates
["yaw"] The car's yaw angle in the map
["speed"] The car's speed in MPH

#### Ego car's previous path data 
This returns the previous list but with processed points removed:
["previous_path_x"] The previous list of x points previously given to the simulator
["previous_path_y"] The previous list of y points previously given to the simulator
["end_path_s"] The previous list's last point's frenet s value
["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data
This provides a list of all other car's attributes (no noise) on the same side of the road. 
["sensor_fusion"] A 2d vector of cars and then that car's data : 
[car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates]. 

---
## Description of the Path Planner
The path planner is initialized by the main.cpp with the map of the highway. Based on the data from the simulator, the path planner creates an instance of the ego vehicle and its neighbouring vehicles using the following method:
```
void PathPlanner::ReceiveData(json data)
```
#### Safety 
The desired behaviour expected from the car is driving in its lane with maximum speed. However, the most important constraint which should be met all times, is collison avoidance. Accordingly, the path planner checks the safety distance of the ego vehicle within its current lane during the planning horizon. If it finds that the safety distance migh be violated in its time horizon, it will consider lane changing. So, it checks the available gap in the left and right lane. If the gap is not enough for overtaking, it will slow down and stay in its current lane. These are investigated in:
```
vector<bool> PathPlanner::SafetyCheck()
```
Than, decision about the behaviour of the ego vehcile is made using the safety check results at:
```
double PathPlanner::NextAction(vector<bool> safety)
```
    
#### Trajectory
Using the output of the NextAction, a trajectory is generated to keep the car in its desired lane and speed employing spline function. To initilize the spline curve, the last two points of the previous trajectory and three points at a far distance have been considered after being transformed (shift and rotation) to the local car coordinates.Then, the actual future path points of the ego vehicle are derived from the spline. In order to avoid abrupt changes of the velocity, the speed changes gradully between different waypoints. These are done in the following method:
 ```
    json PathPlanner::PathPoints(double speed_change)
  ```

---
## Result
- The car drives according to the speed limit.
- Max Acceleration and Jerk are not Exceeded.
- No collision was detected. 
- The car stays in its lane, except for the time between changing lanes.
- The car is able to change lane in a safe manner.
[Screenshot](../result.jpg)


---
## Details
1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

---
# Compiling and executing the project

#### Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
#### Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
Here is the data provided from the Simulator to the C++ Program



