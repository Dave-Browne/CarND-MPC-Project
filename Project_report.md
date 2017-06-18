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

### MPC preprocessing and Polynomial fitting

In `main.cpp` you will find that the pre-processing step for latency has been commented out. This step predicts the state of the vehicle at time = 0.1s (time of latency). Using this method resulted in the car oscillating alot and I wasn't able to correct it by tuning the N, dt and the cost contributing equations. For this reason it is commented out.

>           // Accomodate for latency - predict px, py, psi & v at time latency
>           // double latency = 0.1;
>           // double Lf = 2.67;
>           // px += v * cos(psi) * latency;
>           // py += v * sin(psi) * latency;
>           // psi += v * steer_value * latency / Lf;
>           // v += throttle_value * latency;

The following code translates the location of the car and it's waypoints from global coordinates to vehicle coordinate system (ie x, y and psi are 0). The waypoints `ptsx, ptsy` equations are derived using geometry. See [here](https://discussions.udacity.com/t/mpc-car-space-conversion-and-output-of-solve-intuition/249469/11) for an excellent diagram.

>          // Translate the ref traj from global to vehicle coords
>          // Vehicle is at (px, py). To change to vehicle coords subtract px from x and py from y
>          // Rotate the axis so that x is in direction car is facing, y is 90deg to the left.
>          for (size_t i=0; i<ptsx.size(); ++i) {
>            double xn = ptsx[i] - px;
>            double yn = ptsy[i] - py;
>            ptsx[i] = xn * cos(psi) + yn * sin(psi);
>            ptsy[i] = yn * cos(psi) - xn * sin(psi);
>          }
>          px -= px;
>          py -= py;
>          psi -= psi;

### Latency
Latency is dealt with not by predicting the vehicle state by the latency time, but rather by using a `dt` value > latency time.

