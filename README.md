# CarND Path Planning Project
Self-Driving Car Engineer Nanodegree Program

|Lane Changing Example|Full Cycle (4.25 Miles) Example|
|----|----|
|![](./images/sample2.png)|![](./images/full_cycle.png)|
   
### Description
In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.

The car's localization and sensor fusion data will be provided by the simulator. There is also a sparse map contains a list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too.

The ego car **should avoid** hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

__More technical details please check [writeup](./writeup.pdf)__.

## Simulator

### Usage

Download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on macOS/Linux,
```shell
sudo chmod u+x term3_sim.x86_64
./term3_sim.x86_64
```

#### Some Further Details 

1. The ego use a perfect controller and will visit every (x,y) point it recieves in the list every **0.02** seconds.
2. The units for the `[x, y]` points are in meters and the spacing of the points determines the speed of the car.
3. The vector going from a point to the next point in the list dictates the angle of the car. 
4. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration.
5. The `[x, y]` point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3.
   - NOTE: As this is BETA, these requirements might change. Also currently jerk is over a `0.02` second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.
6. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition.
   - `previous_path_x`, and `previous_path_y` can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Build

### Docker One Liner
Using docker is recommended. Check this [official guide](https://docs.docker.com/engine/install/).

```shell
cd YOUR_CLONE_PATH
docker build -t pathplanningserver:latest .
docker run -p 4567:4567 pathplanningserver:latest
````

### Local Build
#### Dependencies
* cmake >= 3.5
  * [Installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [Install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: [Installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc/g++ is installed by default on most Linux distros
  * Mac: [Install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh`, `install-m1-mac.sh`, or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```shell
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
* [spdlog](https://github.com/gabime/spdlog) >= 1.9
  * Install from source if you have version issue with `fmt/bundle`, otherwise install it from package manager.
    ```shell
    git clone https://github.com/gabime/spdlog.git
    cd spdlog && mkdir build && cd build
    cmake .. && make install
    ```

##### macOS
TODO

##### Debian/Ubuntu

```shell
apt install python3-dev pybind11-dev libfmt-dev libspdlog-dev libboost-dev libgtest-dev
```

##### Other libraries shipped with the repo
- spline: a really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.
- Eigen 3.3

#### Instructions
```shell
git checkout https://github.com/kunlin596/PathPlanning.git
cd PathPlanning
cmake -DCMAKE_BUILD_TYPE=Release -S . -B build
cmake --build build -j 16 --target pathplanningserver 
```

#### Run
The simulator should be lauched simultaneously with the path planning server. Use the command below to lauch the path planning server.
```shell
./build/pathplanningserver --conf default_conf.json -m data/highway_map.csv --loglevel=info
```

### Input Data

The input data to the path planner server is described as below.

#### Map Data
The map of the highway is in `data/highway_map.txt`.

Each waypoint in the list contains `[x, y, s, dx, dy]` values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

#### Ego's localization
There is no noise in the localization result.
The format is described as below,
```
- x: The car's x position in map coordinates
- y: The car's y position in map coordinates
- s: The car's s position in frenet coordinates
- d: The car's d position in frenet coordinates
- yaw: The car's yaw angle in the map in degree
- speed: The car's speed in MPH
```

#### Previous path data
The previous path contains the waypoints haven't been consumed by the simulator from the previous path sent to the simulator.
The format is described as below,
```
- previous_path_x: The previous list of x points previously given to the simulator
- previous_path_y: The previous list of y points previously given to the simulator
```

#### Previous path's end s and d values 
```
- end_path_s: The previous list's last point's frenet s value
- end_path_d: The previous list's last point's frenet d value
```

#### Perception Data
`sensor_fusion` A 2d vector of cars and then that car's id, kinematics, etc.
Each row contains,
```
[
  car's unique ID,
  car's x position in map coordinates,
  car's y position in map coordinates,
  car's x velocity in m/s,
  car's y velocity in m/s,
  car's s position in frenet coordinates,
  car's d position in frenet coordinates
]
```

## Code Style
The python code is formated using `black`.
The cpp code is formated using clang format using a slightly modified version of `Mozilla` coding standard.

## References
- Werling, J. Ziegler, S. Kammel, and S. Thrun, "Optimal trajectory generation for dynamic street scenarios in a frenet frame," in ICRA, 2010, pp. 987–993.
