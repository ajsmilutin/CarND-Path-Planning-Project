# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project, the goal was to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 
The car's localization and sensor fusion data are provided and there is also a sparse map list of waypoints around the highway. 
The car tries to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. 
The car tries to avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times unless going from one lane to another. 
The car is able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it takes little over 5 minutes to complete 1 loop. 
The car never experiences total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

### The planner
The path planner is quite a simple one. The basic planning loop is following:
1. Read the sensor fusion data 
2. Calculate maximal and minimal speed for all lanes
3. If some lane has higher maximal speed than current lane and maximal speed is greater than minimal speed, change the desired lane
4. Create the path that car should follow using car state and desired lane and desired target speed (maximal speed for the desired lane)  

All planning and calculation are done within some time horizon that will be denoted as *th*. 

#### Calculating maximal and minimal speed for all the lanes
The maximal speed for the lane is calculated based on the sensor fusion data of the cars *in front* of our car. If the sensed 
car has greater *s* coordinate than our car it is considered to be in front of us. To calculate the maximal and minimal speed
constant acceleration is assumed. There is some minimal distance that should be between the cars.
For the maximal speed that each car in front allows us to drive at must hold:
 *  *Our car is exactly at the minimal distance (represented in s coordinate) from the car in front of us, at the end of the time horizon *th**
  
This way we get the maximal speeds that each car in front of us allows us to drive at. The maximal speed for the lane is the minium of all maximal speeds for a lane. Of course, the maximal speed for each lane cannot exceed 50mph.

Similarly, we get the minimal, speed but we consider the things in other direction. For minimal speed that each car behind us allows
 us to drive at must hold: 
 * *The car is exactly minimal distance (represented in s coordinate) behind as at the end of the time horizon th*
 
 Now the minimal speed for the lane is the maximum of all minimal speeds that each car allows us to travel at. 
 
 If the minimal speed for a lane is greater than maximal speed for a lane then we can't change to that lane. If the same
 hold for our current lane, we continue at maximal speed and hope that car behind doesn't hit us.
  
  This is implemented in function ```PathPlanner::setFusionData()```
  
#### Creating the path 
All paths are created in XY space using 6th order polynomials. To calculate them, initial and final position, velocity, 
and acceleration are needed. Initial positions, velocities, and accelerations are quite easy to calculate from the vector
of data previously sent to the simulator. 

It is noted that the previous values that simulator sends back to our program have a precision of 3 digits after zero, so the minimal change between two consecutive positions is +/- 0.001. That seems
okay, but if the acceleration is calculated from 3 consecutive samples, it has a resolution of 2.5 which is too coarse. For that reason, the previously sent data is also maintained within ```PathPlanner``` class. 
  
The final position, velocity, and acceleration, needed to find the polynomials, are a bit trickier to calculate. To do so
  first, the *s* and *d* position, velocity and acceleration for the final point are calculated, and then transformed to 
  XY coordinates.
    
Final *s* position is calculated so that car achieves target speed in minimal time. Once again, acceleration is assumed to 
  be constant during the time horizon. If the car can't achieve target speed within time horizon, the final  speed is set to be maximally achievable ( by using maximal acceleration). If car can achieve target speed within time horizon
  final *s* velocity is equal to target velocity and final *s* acceleration is equal to 0.
  
Final *d* position is calculated according to the desired lane. The car should change the lane at constant *d* velocity   so that the lane change is performed within single time horizon. 
  
This part of the code is implemented in ```PathPlanner::nextXY```
 
After the polynomials in X and Y direction are found, new waypoints need to be generated. That is performed by function
```PolynomialXY::evaluate_safe``` . That functions doesn't evaluate polynomial directly, but evaluates acceleration of the
polynomial and integrates it twice. That way we can have closer control of total acceleration and total speed. If somehow
total acceleration exceeds the maximum, it is scaled back to be below maximum allowed acceleration. The Same thing is applied to velocity.

### Conclusion
It was the quite challenging project, and I've had all sorts of problems. The main problem was that during some very short periods
of time the planned path was violating given constraints. There were two major causes to this:
1. The inaccuracy of returned previous waypoints
2. "Unsafe" evaluation of polynomial

After identifying and fixing those two issues, the car now drives great!
