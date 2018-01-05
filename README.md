# CarND MPC Project
Author: *Igor Passchier*

Email: *igor.passchier@tassinternational.com*

## Introduction
This Readme follows the rubric of the project, which can be found at [here](https://review.udacity.com/#!/rubrics/896/view)

## Compilation
### Your code should compile
The code has been developed on a mac with latest MacOS and latest Xcode. The program can be compiled and run when all dependencies are met via:
* mkdir build
* cd build
* cmake ..
* make
* ./mpc

## Implementation
### The model
The controller runs everytime a message from the simulator is received, see line 36 of [main.cpp](src/main.cpp). It first reads all relevant input from the received message. On line 73, it transforms the trajectory to vehicle coordinates via the function unityToCar, which is implemented in [helper_functions.cpp](src/helper_functions.cpp#L54).

An issue with the Eigen library prevents to initialize directly the VectorXd based on the transformed values in next_x_vals, so therefore the loop on [line 76](src/main.cpp#L76).

On line 81, the trajectory is fit with a 3d polynomial, the function itself is implemented in helper_functions.cpp and has been provided in class.

The state of the vehicles consists of:
* x (x position in car coordinates)
* y (y position in car coordinates)
* psi (driving angle in car coordinates)
* v (velocity of the car)
* cte (offset in y-direction with respect to reference trajectory)
* epsi (angular offset wit respect to the reference trajectory)
As all are calculated in vehicle coordinates, the first 3 are 0, v is given from the message, and cte and epsi are obtained from the fit of the reference trajectory in vehicle coordinates.

On line 86, the solver is called to find a best solution, see later for an explanation of the solver.

The solver returns the steer and throttle values, and a list of x/y values of the calculated solution. These are obtained from the answer of the solver, and put in the json message ([lines 87-122](src/main.cpp#L87-L122)).
To account for actuator delay, a sleep is introduced on line 133. This is parameterized with a constat delay_steps, which is set in [MPC.h](src/MPC.h#L11).

On line 135, the resulting command is send back to the simulator.


### Timestep length and Elepsed Duration

### Polynomial Fitting and MPC Preprocessing

### Model Predictive Control with Latency

## Simulation
### Vehicle drives a lap successfully
With the provided code and the term2 simulator, the car drives one and more laps successfully.

