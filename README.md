# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Model

The project uses MPC controller to control the vehicle on the track. The model of MPC controller consist of current state and several predicted ones. Based on this data the model tries to fit the model to reduce the cost and provide the best trajectory.

The state includes: position `(x, y)`, velocity `(v)`, acceleration `(a)`, vehicle angle relative to map X axis `(psi)`, vehicle steering angle `(delta)`, cross-track error `(cte)` and angle of the vehicle relative to track direction `(epsi)`.

The actual state for the predicted steps is depends on reference state and the actuators applied. Actuators(acceleration, steering) adjustment allows to improve the model cost and result in more optimized trajectory. To update the state, the following equations are used:

    x   = x + v * cos(psi) * dt
    y   = y + v * sin(psi) * dt
    psi = psi + (v / Lf) * delta * dt
    v   = v + a * dt
where the `dt` is the chosen time delta for the state prediction.  

To improve the actuators influence on the car, the cost should be defined properly. The most important factor is the **cte** which ensures that car is following the track. Nevertheless, the **epsi** metric is important as well to ensure that right delta is applied and the car direction does not vary a lot. The other cost parts, as actuators themselves or actuators delta are added to remove unnecessary fluctuations. To ensure movement forward, the **velocity** difference to target velocity is added as well.

## Timesteps

To choose right values for the time step calibration (`N` (amount of steps) and `dt` (time delta between each step)) two things should be considered: precision and performance. Smaller `N` improves performance, whereas smaller `dt` improves precision. However, among this factors, the distance of the predicted track should be considered as well. If the distance is too small, the MPC will perform poorly due to inability to look far enough to the "future".
First values were chosen to be 20 time steps of 0.1 second each. However, the performance was poor, so the amount of the timesteps was decreased to **10** and time delta increased to **0.18**.

## Preprocessing

Before fitting polynomial and passing values to the MPC, the x and y values are translated to fit car coordinate system, which gives easier resolution for the further calculations. First, the coordinates were translated by `(-x, -y)` and then rotated by `psi`.
