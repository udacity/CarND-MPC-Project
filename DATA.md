# Websocket Data

This document describes the JSON object send back from the simulator command server.

Fields:

* ptsx (Array<float>) - The global x positions of the waypoints.
* ptsy (Array<float>) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity
since y is the up-down direction.
* psi (float) - The orientation of the vehicle in **radians** converted from the Unity format to the standard format expected in most mathemetical functions (more details below).
* psi_unity (float) - The orientation of the vehicle in **radians**.
* x (float) - The global x position of the vehicle.
* y (float) - The global y position of the vehicle.
* steering_angle (float) - The current steering angle in **radians**.
* throttle (float) - The current throttle value [-1, 1].
* speed (float) - The current velocity in **mph**.


### Unity Degree Conversion

The degree representation does not correspond to the following representation below, standard in most mathematical functions. `psi` is the mapping to the below orientation representation.

```
//            90
//
//  180                   0/360
//
//            270
```
