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
The controller runs everytime a message from the simulator is received, see line 36 of [main.cpp](src/main.cpp#L36-L145). It first reads all relevant input from the received message. On line 73, it transforms the trajectory to vehicle coordinates via the function unityToCar, which is implemented in [helper_functions.cpp](src/helper_functions.cpp#L54-L64).

An issue with the Eigen library prevents to initialize directly the VectorXd based on the transformed values in next_x_vals, so therefore the loop on [line 76](src/main.cpp#L74-L79).

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

On [lines 135](src/main.cpp#L135), the resulting command is send back to the simulator.

The MPC model and solver are implemented in [MPC.cpp](src/MPC.cpp). The variables are initialized and the constrain limits are defined in [MPC::Solve](src/MPC.cpp#L154-L331). The state consists of 6 variables, resulting in 6N parameters, and 2 actuators, resulting in 2*(N-1) parameters. The constraints are based on all state variables on all steps, so also 6N constraints. These values are set in [line 159-163](src/MPC.cpp#L159-L163). All variables are initilized to 0, except for the first that are set to the current state (line 167-176). The bounds are effectively completely open for the state variables (line 180-205), and limited to -1..1 for a and -25deg..25 deg for delta (line 207-214).

All constraints are defined such that they should be 0, expect for the inital states. These bounds are specified in line 218-237. After setting all options, the solver is called in [line 264](src/MPC.cpp#L264-L265). Based on the solution, the inidividual contributions to the cost and some other parameters are calculated and printed for debugging (line 279-316).

The actuator values and the path are stored in a return vector and returned to the main loop(line 323-330).

The actual dynamic model is implemented in [FG_eval operator ()](src/MPC.cpp#L38-L146). The cost function is fg[0] and loops over all timesteps. The setspeed is depending on the curvature of the road. Initially I tried to make it depending on the steering angle, but that results in a less stable controller. The curvature is calculated from the parametrization of the road in [line 57-62](src/MPC.cpp#L57-L62). This function is based on a formula from https://www.math24.net/curvature-radius/ :K=abs(y'')/(1+y'^2)^(3/2). 

The cost function is based on a cost related to cte, epsi, v, delta, a, and the derivatives of delta and a, and is calculated in lines 66-85. The relative weight is controlled by constants, that are set on line 21-22. the cost contributions of cte and epsi have been tuned to be from the same size of magnitude, the speed contribution some what smaller. To stabilize the controller, a cost is added related to the steering angle. To stabilize in case of a delay, the two terms related to the derivatives are added. If only a smaller change is allowed, the effect of the delay is less. A cost related to a is not really necessary, as that does not influence the performance. Of course, if it was a real car, it would be more comfortable to add some cost to the acceleration itself as well.

The dynamic model is implemented on [line 97-145](src/MPC.cpp#L97-L144). To account for the actuator delay, the actuator values of an earlier step is taken (line 121-128). Two changes have been made compared to the dynamic model described in the lessons.

1. An additional factor 10 is added to the actual actuator setting. It is unclear what the conversion factor from the actuator setting and the actual acceleration si. The factor 10 gave a good result.
2. A factor is added for the drag, proportional to v^2. The actual value has been determined by checking what the speed saturated to for a specific value of the actuator. For an actuation of 0.416, this turned out to be roughly 48 Mph, so that why these values have been choosen. I have observed that at larger speeds, this is not completely correct, so this could be improved, but it is sufficiently accurate to get the motion nicely stable.

The following equations are implemented in lines 136-143:
* x1=x0+v\*cos(psi)\*dt
* y1=y0+v\*sin(psi)\*dt
* psi1=psi0+v/Lf\*delta\*dt
* v1=v0+a\*dt
* cte1=(f(x0)-y0)+v\*sin(epsi)\*dt
* epsi1=(psi0-g(x0))-v/Lf\*delta\*dt

Here, f(x0) is the y coordinate calculated from the parametetrization at x0. g(x0) is the direction of the road calculated from x0. g(x0) is called psides0 in the code.



### Timestep length and Elapsed Duration

### Polynomial Fitting and MPC Preprocessing

### Model Predictive Control with Latency

## Simulation
### Vehicle drives a lap successfully
With the provided code and the term2 simulator, the car drives one and more laps successfully.

