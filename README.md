# Path Planning

This project is a C++ implementation of a self driving car path planner for controlling a simulated car in highway traffic.  The simulation program provides the car's state, localization information, map information and sensor fusion data for other cars.  This c++ program implements behavior and trajectory modules of a self driving car to process this information to keep the car driving in traffic, passing other cars when necessary.  The programmed car must stay below speed limit of 50 mph, keep acceleration below 10 m/s^s, and keep jerk below 10 m/s^s^s.  And obviously, the car cannot crash or run off of the road.  A picture of the simulation program is shown below.

<br /><br />
<p align="center">
<img src="https://github.com/TheOnceAndFutureSmalltalker/path_planning/blob/master/img/simulator.JPG" width="802px" /><br /><b>Car on Track Simulator</b></p>
<br />


## Approach

I started with the project walkthrough video which great help for getting started.  The video helped me in basic controlling of vehicle.  It provided such nuggets as:
  * How to capture and interpret all of the state and sensor fusion data from simulator
  * How to interpret waypoints map data from file
  * How to add and use the spline library to generate trajectories
  * How to generate lane passing trajectories
  * How to use anchor points in generating trajectories
  * How to leverage unused path points from previous trajectory to maintain smooth motion
  * How to keep under max speed, acceleration, and jerk 
  * Do everything in Frenet coordinates

This saved me a great deal of time.

From there, I implemented a simple state machine with the following states: Keep Lane, Prepare Lane Change, Lane Change Left, and Lane Change Right.

I only have the one Prepare Lane Change state, as this was all that was needed for this scenario.  When in outer lanes, there is only one option for lane change.  When in center lane, I want to evaluate the two options right and left for the best solution.  This simple approach falls apart, of course, once we introduce a desire to be get in a specific lane, for example, get into the far right lane to take an off ramp.  For this project, however, we just want to get into whatever lane is moving the fastest.

There is logic for transitioning from one state to the next - main.cpp, lines 449-516.  Then I have logic for executing the current state.  This mostly involves changing speed and lane number.  This is found at lines 520-568 of main.cpp.  After this, I calculate a new trajectory and send it to the simulator, main.cpp lines 572-680.  

I wrote a few helper functions to make things easier.  First, I wrote carInFront() which looks at my car's state and the sensor fusion data and returs speed and distance of the car directly in front of me.  This information is used in most decision making logic throughout my program.  This function is found in main.cpp, lines 180-206. 

Next I wrote followThatCar() which takes my car's state and state of car in front of me and adjusts my velocity so that I stay a safe constant distance behind it.  This is used in Keep lane and Prepare lane Change states.  The function is found in main.cpp, lines 288-329.

The most interesting and useful function I wrote was scoreLaneChange().  This functions takes in my car's state, the sensor fusion information, my lane and a target lane for changing into.  It returns a score indicating benefit of the change.  It implements several rules comparing my car to cars in target lane and the car in front of me.  It is found in main.cpp, lines 209-285. 

## Analysis of My Approach 

I only consider cars to be points - have no length or height.  This works because all of my longitudinal buffer distances are more than a car length.  It fails, however, when another car veers into my lane and side swipes me.

I am not using prediction, everything is based on current state.  Again, this works for this scenario as long as everyone is well behave.  Yet, there are corner cases where it fails.  These are infrequent.

I am not using cost functions, per se, as described in lessons, but my code applies much of the same logic for decision making.  I am just merely implementing simple rules.  And again, it works for the simple scenario of this project.

My lane change strategy is very naive.  Once I get stuck behind a slower car, I just wait until its clear in one of the lanes next to me and move into that lane.  I end up loosing out on some passing opportunities.  For example, I do not adjust my speed to come along side a gap in one of the other lanes.

I have many parameters that could use further tuning, but this is very time consuming.  I could spend weeks doing that!  The current parameter values seem to work OK.

The code itself is documented, but not well organized.  Its more of a prototype




# Results

The car stays within speed, acceleration, and jerk parameters and passes smoothly when necessary.  The change lane strategy is quite passive however.  The car just waits until an opening comes up, it does not try to position itself for a change lane by slowing down, etc.  Also I do not handle the scenario where I am in the right hand lane stuck behind a slow car and the center lane is blocked as well, but the far left lane is wide open.  My strategy only considers adjacent lanes for lane change.

# Conclusions

There are several more subtle rules I would like to implement and further tune the parameters that I have.  Tuning for specific cases is hard to do since the simulation is random and the particular case you are interested in might not come up after several runs of the simulator.  It would be nice to have a configurable simulation program to conjure up specific scenarios for testing.

I would also like to try using more of the techniques covered in the lesson like prediction and cost functions.  My approach, while working fine for the limited scenario provided by the simulation program, will soon become inadequate in a real world scenario.

