# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Goals
Project description:

Complete project implementation is coded in the main.cpp source file.
There is an object that holds the complete system state (line 20 to 105),
and is organized in group of variables and constants, it is great to have 
everything in one single place, so no parameters are required to be passed
to functions and makes things more simple to integrate with other modules.

Object detail:
struct system_state {

  const int SENSOR_FUSION_ID = 0;
  const int SENSOR_FUSION_X = 1;
  const int SENSOR_FUSION_Y = 2;
  const int SENSOR_FUSION_VX = 3;
  const int SENSOR_FUSION_VY = 4;
  const int SENSOR_FUSION_S = 5;
  const int SENSOR_FUSION_D = 6;

  const int NEXT_CAR_DISTANCE = 0;
  const int NEXT_CAR_SPEED = 1;
  const int PREV_CAR_DISTANCE = 2;
  const int PREV_CAR_SPEED = 3;

  // Behaviours
  int cycle_counter = 0; //Update behaviour every 50 cyles
  int cycles_per_bejaviour_update = 10; //Update behaviour every 50 cyles
  double prev_car_distance_cost_coef = 0.010;
  double prev_car_speed_cost_coef = 0.06;
  double next_car_distance_cost_coef = 0.025;
  double next_car_speed_cost_coef = 0.06;  
  double min_cost = 0.3;
  int is_changing_left_lane = 0;
  int is_changing_right_lane = 0;

  double target_x = 40.0;
  double initial_spline_points_spacing = 40.0;
  int total_path_points = 50;
  double cycle_period = 0.02; // car starts at slowest lane, most left lane is 0, total number of lanes is 3
  int lane = 1; // car starts at slowest lane, most left lane is 0, total number of lanes is 3
  double lane_change = 1; // Incremental change lane
  double lane_change_inc = 0.01; // Change lane increment
  int target_lane = 1;
  double lane_size = 4.0; // lane width
  int total_lanes = 3; // car starts at slowest lane, most left lane is 0, total number of lanes is 3

  // Physics
  double safe_car_distance = 50.0; // m
  double safe_speed_diff = 5.0; // miles/h
  double safe_min_speed = 15.0; // miles/h  
  double critic_car_distance = 10.0; // m
  double top_velocity = 49.5; // miles/h (Top velocity is 50 m/h)
  double reference_velocity = 0.0; // miles/h 
  double low_velocity_inc = 0.4; // m/s^2 
  double high_velocity_inc = 1.0; // m/s^2 
  double next_car_distance = 1000000.0; // m
  double next_car_speed = 1000000.0; // m/s

  double next_s = 0.0; // next s point for car position
  double next_d = 0.0; // next d point for car position
  double next_x = 0.0; // next x point for car position
  double next_y = 0.0; // next x point for car position

  // Car will be referenced from started point or from the last point of previous path
  double car_s = 0.0; // reference s car position
  double ref_x = 0.0; // reference x car position
  double ref_y = 0.0; // reference y car position
  double ref_yaw = 0.0; // reference car angle

    // Map data
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Previous path
  int prev_size = 0;
  vector<double> previous_path_x;
  vector<double> previous_path_y;

  // Future points to create a spline
  vector<double> future_spline_points_x;
  vector<double> future_spline_points_y;

  // Sensor fusion
  vector<vector<double>> sensor_fusion;  

  // Path spline
  tk::spline path_spline;

  // next values for path
  vector<double> next_x_vals;
  vector<double> next_y_vals;
} _ppss; //Path Planner System State


The main entry point function is calculatePath() (line 711):

void calculatePath() {
  initState(); // Initialize variables and car ref for current cycle
  evaluateBehaviour(); // Select speed and lane based on cost functions  
  controlCurrentLane(); // Get next car and keep a safe distance while in current lane
  calculateNextPoints(); // Geometric calculation of path based on lane and ref speed
}

This function calls:
void initState() (line 287) which not only initialize variables values 
but also create the spline required for the path calculations.
Additionally it starts the lane changes and increment lane_change value.

Next is void evaluateBehaviour() (line 642) which is called every 10 main cycles,
(in this case 5 times per second). This function evaluate change lane costs only 
if next car distance is smaller than twice the safe distance (defined in the main state object),
and if the car is not already changing lanes, signaled with a coupe of flags:
_ppss.is_changing_left_lane
_ppss.is_changing_right_lane

    // Cost functions
    // Only evaluate change of lane if it makes sense
    if(_ppss.next_car_distance < _ppss.safe_car_distance * 2.0 &&
       _ppss.is_changing_left_lane == 0 &&
       _ppss.is_changing_right_lane == 0) {...}

In case the cost is less than the minimum cost (_ppss.min_cost)
evalChangeLaneCost(...) is called passing the target lane as the argument.

First left change is evaluated and in case it is positive, right evaluation is 
not called, idea is to give priority to left changes.

In case cost is as low as required then lane change is started as follows:
      if(change_left_lane_cost < _ppss.min_cost && _ppss.lane > 0) {
        _ppss.is_changing_left_lane = 1;
        _ppss.is_changing_right_lane = 0;
        _ppss.target_lane = _ppss.lane - 1;
        _ppss.lane_change = _ppss.lane;
        cout << "\033[2J\033[1;1H";
        cout << "<<<<<<<<<<<<<<<<<< Start change lane to the left";
        cout << "target lane:" << _ppss.target_lane << endl;
        cout << "Change lane:" << _ppss.lane_change << endl;
        cout << "lane:" << _ppss.lane << endl;
        cout << "change_left_lane_cost:" << change_left_lane_cost << endl;
        cout << "_ppss.min_cost:" << _ppss.min_cost << endl;
        return;
      }

Change lane evaluation, double evalChangeLaneCost(int target_lane) (line 562) 
function works as follows:
1) Gets prev and next target car variables (if any):
  vector<double> vehicles_variables = evalVehiclesInTargetLane(target_lane);
  double prev_car_distance = vehicles_variables[_ppss.PREV_CAR_DISTANCE];
  double prev_car_speed = vehicles_variables[_ppss.PREV_CAR_SPEED];
  double next_car_distance = vehicles_variables[_ppss.NEXT_CAR_DISTANCE];
  double next_car_speed = vehicles_variables[_ppss.NEXT_CAR_SPEED];

2) If next car goes slower then no sense to change lane, return 1.0 (max cost value)
3) Else 4 costs are evaluated:
  // Evaluate change of lane is not risky with a set of cost functions
  // Distance factor of prev car
  double prev_car_distance_cost = fabs(_ppss.safe_car_distance / prev_car_distance);
  // Speed difference of prev car
  double prev_car_speed_cost = fabs(prev_car_speed - _ppss.reference_velocity) / _ppss.top_velocity;
  // Distance factor to next car
  double next_car_distance_cost = fabs(_ppss.safe_car_distance / next_car_distance);
  // Speed factor to next car
  double next_car_speed_cost = fabs(next_car_speed - _ppss.reference_velocity) / _ppss.top_velocity;


4) Costs are trimmed in case they exceed domain limits:
  if(cost > 1.0) cost = 1.0;
  if(cost < 0.0) cost = 0.0;

5) Eval risks:
  // If previous car speed is too high in comparisson with ego vehicle speed 
  // then abort lane change if any
  double speed_diff = prev_car_speed - _ppss.reference_velocity;

  if(speed_diff > _ppss.safe_speed_diff) {
    cost = 1.0;
  }
  // If speed is too low then avoid changing lanes
  if(_ppss.reference_velocity < _ppss.safe_min_speed) {
    cost = 1.0;
  }


Then void controlCurrentLane() (line 502) is called 
which takes care of the control of the car in the current lane
increasing or decreasing speed as required:

  // If distance of next car in current lane is safe then accelerate up to ref speed
  if(_ppss.next_car_distance > _ppss.safe_car_distance || next_car_index == -1)
    accelerate_soft();

  // If distance of next car in current lane is not safe then accelerate up to ref speed
  if((_ppss.next_car_distance < _ppss.safe_car_distance ||  _ppss.reference_velocity - _ppss.next_car_speed < 0)
    && next_car_index > -1)
    if(_ppss.next_car_speed < _ppss.reference_velocity * 0.8)
      de_accelerate_soft();    

  // If distance of next car in current lane is not safe then accelerate up to ref speed
  if(_ppss.next_car_distance < _ppss.critic_car_distance && next_car_index > -1) {
    cout << "Critic distance" << endl;
    de_accelerate_hard();    
  }      

  In particular to keep things confortable for the "virtual passenger" 
  de_accelerate_soft() function considers difference in speed with the car in 
  front,si its decrement amount is smaller if the speed diff is smaller.

  Finally to avoid collisions de_accelerate_hard() is called in case distance is less than 
  the critical distance.

  The last function is void calculateNextPoints() (line 442), thsi function simply 
  calculate next points based on spline already parametrized in the initialization function:

   double target_x = _ppss.target_x;
   double target_y = 0;
   double x_offset = 0.0;

   if(_ppss.future_spline_points_x.size() > 2 || _ppss.previous_path_x.size() > 2)
    target_y = _ppss.path_spline(target_x);
    
   double target_distance = distance(target_x, target_y, 0, 0);

   // Fill the rest of the path
   for(int i = 1; i <= _ppss.total_path_points - _ppss.prev_size; i++) {
    double steps_count = target_distance / (_ppss.cycle_period * _ppss.reference_velocity / 2.24);
    double next_x_point = x_offset + target_x / steps_count;
    double next_y_point = _ppss.path_spline(next_x_point);

    x_offset = next_x_point;

    // Rotate back to world coord system
    vector<double> world_next_point = trasnformToWorldCoords(next_x_point, next_y_point);

    _ppss.next_x_vals.push_back(world_next_point[0]);
    _ppss.next_y_vals.push_back(world_next_point[1]);
   }

Detail of speed control funtions:
void accelerate_soft() {
  _ppss.reference_velocity += _ppss.low_velocity_inc;
  if(_ppss.reference_velocity > _ppss.top_velocity)
    _ppss.reference_velocity = _ppss.top_velocity;  
}

void de_accelerate_soft() {
  double speed_diff = _ppss.reference_velocity - _ppss.next_car_speed;

  // Make inc smaller when speed difference is smaller between ego car and next car
  double inc = _ppss.low_velocity_inc * (speed_diff) / (_ppss.reference_velocity * 1.5);

  if(inc > _ppss.low_velocity_inc)
    inc = _ppss.low_velocity_inc;

  _ppss.reference_velocity -= inc;
  if(_ppss.reference_velocity < 0.0)
    _ppss.reference_velocity = 0.0;  
}

void de_accelerate_hard() {
  _ppss.reference_velocity -= _ppss.high_velocity_inc;
  if(_ppss.reference_velocity < 0.0)
    _ppss.reference_velocity = 0.0;  
}

void accelerate_hard() {
  _ppss.reference_velocity += _ppss.high_velocity_inc;
  if(_ppss.reference_velocity > _ppss.top_velocity)
    _ppss.reference_velocity = _ppss.top_velocity;  
}



#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

