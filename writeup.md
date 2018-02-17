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

The code implements the model predictive control mechanism for driving around an emulated test car in the Udacity simulator. As input parameters the program receives the coordinates of the middle waypoints of the road `ptsx` and `ptsy` (where the crosstrack error is zero), the heading angle `psi`, current velocity `v`, current position `px` and `py` and the current actuator values, namely the steering angle `delta0` and the throttle amount `a0`.

From the current state the model calculates the following `N` interations of expected future states (including the postition, heading, velocity, and the appropriate actuator values (a and delta). This is done by solving the vehicle model equations:

![alt text][image5]

The kinematic equations are based on a kinematic car model. In order to model the future stated we also need tu build a cost function to penalize unwanted states/state transitions or actuations. This will be extended by certain constraints, lower and upper bounds. In the end a solver will be used to find the optimal actuator inputs that minimize the cost. The result is the current projected optimal MPC trajectory.

### Timestep Length and Elapsed Duration (N & dt)

In the final version of the project I used `N = 10` iterations and `dt = 0.25` seconds time delta.

#### Number of Iterations - the `N` hyperparameter

This was based on mostly empirical observations. If I decreased the `N` parameter (`N = 5`) then a not totally suitable polynom were fitted onto the waypoints, but CPU-load would be decreased. This, however, is not needed - the 10 iterations per state is still feasable. If I increased `N` parameter (`N = 18`) that resulted in higer CPU-load and no noticable benefit on curve fitting - instead resulting in a higher latency due to high load on the CPU.

#### Time Delta Length - the `dt` hyperparameter

Now let's take a look at the time delta. More frequent iterations (`dt = 0.05`) result in a less precise trajectory prediction, thus forcing the vehicle into sudden braking and/or sharp steering angle changes. On the other hand increasing the value (`dt = 0.35`) tends to predict the future unprecisely as well.

### Polynomial Fitting and MPC Preprocessing

Received waypoints are first transformed into the vehicle coordinate system, then a 3rd degree polynom is fitted onto them using [this](https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716) procedure. Furthermore all applicable state values are transformed into the vehicle's coorinate system:
```c++
x0 = 0
y0 = 0
psi0 = 0
v0 = v
cte = coeffs[0]
epsi = 0 - atan(coeffs[1])
```
Notice that cross-track error and psi error rely only on the first and the second coefficient of the resulting 3rd degree curve - this leads back us to the benefit of the simplified state: in the car coordinate system the position and the heading angle is all zero.

### Model Predictive Control with Latency

There is a 100ms latency introduced into the model. The original intention was to simulate real-life situations when it takes certain amount of time for the actuator outputs to take effect. This can be handled by adjusting the initial state using the latency value as dt. Again, the equations from "The Model" part come in handy here. In code this looks the following:

```c++
const double cte = coeffs_[0] - 0;         // all other coeffs would become 0
const double epsi = 0 - atan(coeffs_[1]); // adjusted psi is also 0
const double psi_corr = v * delta0 / Lf * LATENCY;

// Set the initial state values incorporating latency
state_ << 0 + v * LATENCY,
          0,
          0 + psi_corr,
          v + a0 * LATENCY,
          cte + v * sin(epsi) * LATENCY,
          epsi + psi_corr;
```

Another possible solution would be having `dt` and `LATENCY` share the same value, e.g. `dt = 0.1 secs` and simply shifting resulting actuations backwards by one (i.e. for timestep `t` the actuation values of `t - 1` are taken).

## Simulation
### The vehicle must successfully drive a lap around the track.

The vehicle is able to drive on the test track safely while maintaining a fairly high speed.

![alt text][image2]

I run several tests in the simulator. I recorded two full laps and uploaded the results to Youtube:

[![alt text][image4]](https://youtu.be/PNpAR9-Sl5Y)
