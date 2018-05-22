# Path Planning

This project is a C++ implementation of a self driving car path planner for controlling a simulated car in highway traffic.  The simulation program provides the car's state, localization information, map information and sensor fusion data for other cars.  This c++ program implements behavior and trajectory modules of a self driving car to process this information to keep the car driving in traffic, passing other cars when necessary.  The program must stay below speed limit of 50 mph, keep acceleration below 10 m/s^s, and keep jerk below 10 m/s^s^s.  And obviously, the car cannot crash or run off of the road.  A picture of the simulation program is shown below.

<br /><br />
<p align="center">
<img src="https://github.com/TheOnceAndFutureSmalltalker/mpc_project/blob/master/img/simulator.JPG" width="802px" /><br /><b>Car on Track Simulator</b></p>
<br />


# Approach

I started with the project walkthrough video which was very help for getting started.

The video helped me in basic controlling of vehicle.  It provided such nuggets as
How to capture and interpret all of the state and sensor fustion data from simulator and waypoints map data from file
using spline library to generate trajectories.  
How to use anchor points in generating trajectories
How to leverage unused path points from previous trajectory,
How to keep under max speed, acceleration, and jerk 

This saved me a great deal of time.

cars are points, I am not considering length/width of cars

I am not using prediction, everything is based on current state

I am not using cost functions, per se, as described in lessons, 
but my code applies much of the same logic for decision making



From there, I implemented a simple state machine with the following states: Keep Lane, Prepare Lane Change, Lane Change Left, and Lane Change Right.

I only have the one Prepare Lane Change, as this was all that was needed for this scenario.  When in outer lanes, there is only one option for lane change.  When in center lane, I want to evaluate the two options right and left for the best solution.  This simple approach falls apart, of course, once we introduce a desire to be get in a specific lane, for example, get into the far right lane to take an off ramp.  For this project, we just want to get into whatever lane is moving the fastest.

There is logic for transitioning from one state to the next.  Then I have logic for executing the current state.  This involves changing speed and lane number, and then calculating a new trajectory.


# Results

The car stays within speed, acceleration, and jerk parameters and passes smoothly when necessary.  The change lane strategy is quite passive however.  The car just waits until an opening comes up, it does not try to position itself for a change lane by slowing down, etc.  

# Conclusions

There are several more subtle rules I would like to implement and further tune the parameters that I have.  Tuning for specific cases is hard to do since the simulation is random and the particular case you are interested in might not come up after several runs of the simulator.  It would be nice to have a configurable simulation program to conjure up specific scenarios for testing.

I would also like to try using more of the techniques covered in the lesson like prediction and cost functions.  My approach, while working fine for the limited scenario provided by the simulation program, will soon become inadequate in a real world scenario.

