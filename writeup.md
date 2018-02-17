# CarND-MPC-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[image1]: ./img/comp_mpc.png
[image2]: ./img/MPC_Udacity_test.jpg
[image3]: ./img/MPC_Udacity_test_2.jpg
[image4]: ./img/youtube.jpg
[image5]: ./img/state_eq.png


---

*I'm going thru all the rubric points here.*

## Compilation
### Your code should compile.

It does.

![alt text][image1]

## Implementation
### The Model

The code implements the model predictive control mechanism for driving around an emulated test car in the Udacity simulator. As input parameters the program receives the coordinates of the middle waypoints of the road (where the crosstrack error is zero), the heading angle, current velocity, current position and the current actuator values, namely the steering angle and the throttle amount.

From the current state the model calculates the following N interations of expected future states (including the postition, heading, velocity, and the appropriate actuator values (a and delta). This is done by solving the vehicle model equations:

![alt text][image5]

### Timestep Length and Elapsed Duration (N & dt)

...

#### The `N` hyperparameter

...

#### The `dt` hyperparameter

...

### Polynomial Fitting and MPC Preprocessing

...

### Model Predictive Control with Latency

...

## Simulation
### The vehicle must successfully drive a lap around the track.

I run several tests in the simulator. I recorded two full laps and uploaded the results to Youtube:

[![alt text][image4]](https://youtu.be/PNpAR9-Sl5Y)
