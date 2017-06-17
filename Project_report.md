## Model Predictive Control

### Model

The model is made up of the vehicle state and actuators. The state vector consists of the x and y positions of the car (`px, py`), the orientation (`psi`), velocity (`v`), cross track error (`cte`) and orientation error (`epsi`).
The actuators are the vehicle steering angle (`delta`) and the throttle (`a`).
The state of the vehicle is updated using the following equations:
> 1) x' = x + v*cos(psi)*dt
> 2) y' = y + v*sin(psi)*dt
> 3) psi' = psi + v / Lf * delta * dt, where Lf is a constant, the distance from the front of the vehicle and it's centre of gravity.
> 4) v' = v + a*dt

### Timestep length (N) and duration (dt)

`N` is the number of timesteps predicted and `dt` is the duration of each timestep.
The total predicted time is known as the Horizon `T = N * dt`. In my code I use N = 10 and dt = 0.15. The total horizon T = 1.5s.
If I predict too far into the future the vehicle does not track the reference trajectory nicely, as it is influenced by turns far ahead.

I tried using values of dt ranging from 0.05 to 0.2. Too small doesn't predict far enough as a higher N value increases compute time. Too large results in lower accuracy in tracking the reference trajectory.
I also tried N values ranging from 7 to 20. A Horizon time of 1s to 1.5s seemed to work well.

### MPC preprocessing
