## Model Predictive Control

### Model

The model is made up of the vehicle state and actuators. The state vector consists of the x and y positions of the car (`px, py`), the orientation (`psi`), velocity (`v`), cross track error (`cte`) and orientation error (`epsi`).
The actuators are the vehicle steering angle (`delta`) and the throttle (`a`).
The state of the vehicle is updated using the following equations:
> 1) x' = x + v*cos(psi)*dt
> 2) y' = y + v*sin(psi)*dt
> 3) psi' = psi + v / Lf * delta * dt, where Lf is a constant, the distance from the front of the vehicle and it's centre of gravity.
> 4) v' = v + a*dt
> 5) cte' = f(x) - y + (v*sin(epsi)*dt)
> 6) epsi' = epsi + v / Lf * delta * dt, where epsi is the difference between the current orientation and desired orientation
>          = (psi - psides) + v / Lf * delta * dt

### Timestep length (N) and duration (dt)

`N` is the number of timesteps predicted and `dt` is the duration of each timestep.
The total predicted time is known as the Horizon `T = N * dt`. Values of N = 10 and dt = 0.15 were used. The total horizon is therefore T = 1.5s. If the prediction is too far into the future the vehicle does not track the reference trajectory nicely, as it is influenced by turns far ahead.

`dt` values ranging from 0.05 to 0.2 were tested. Too small `dt` doesn't predict far enough as a higher N value increases compute time. Too large `dt` results in lower accuracy in tracking the reference trajectory.
`N` values ranging from 7 to 20 were tested. A Horizon time of 1s to 1.5s works well.

### MPC preprocessing and Polynomial fitting

In `main.cpp`, the velocity is converted from mph to m/s. This is done because the velocity received from the simulator is in mph and the waypoints are in meters.

>           v = v * 1600 / 3600;

Secondly, the steering value must be multiplied by -1 to counter-steer the car back towards the reference trajectory.

>           steer_value *= -1;

The following code translates the location of the car and it's waypoints from global coordinates to vehicle coordinate system (ie x, y and psi are 0). The waypoints `ptsx, ptsy` equations are derived using geometry. See [here](https://discussions.udacity.com/t/mpc-car-space-conversion-and-output-of-solve-intuition/249469/11) for an excellent diagram. `px`, `py` and `psi` are now 0 and the mpc solver becomes much simpler.

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

The new coordintates are fitted to a polynomial as using the polyfit function.

>          // Find the polynomial coefficients for the reference trajectory
>          // Polynomial of 3rd order: f(x) = Ax^3 + Bx^2 + Cx + D
>          //             Derivative: f'(x) = 3Ax^2 + 2Bx + C
>          order = 3;
>          auto coeffs = polyfit(ptsx_, ptsy_, order);

### Latency
Latency is dealt with by predicting the vehicle state at the latency time (0.1s). All 6 state variables need to be predicted at the latency time. To do this `px`, `py`, `psi` and `v` were predicted at 0.1s. These were then translated to vehicle coordinates. This new state was then used to calculate `cte` and `epsi`. Since the state is already at 0.1s, `dt = 0` and so the terms `(v*sin(epsi)*dt) = 0` for `cte'` and `v/Lf*delta*dt = 0` for `epsi'`. The resulting equations are shown below.

It seems that running the simulator on higher resolution and quality levels increases computation time and therefore adds additional latency, but it wasn't necessary to account for this in this implementation.

>           Accomodate for latency - predict px, py, psi & v at time latency
>           double latency = 0.1;
>           double Lf = 2.67;
>           px += v * cos(psi) * latency;
>           py += v * sin(psi) * latency;
>           psi += v * steer_value * latency / Lf;
>           v += throttle_value * latency;
>
>           cte = polyeval(coeffs, px) - py;
>
>           derivative = 3*coeffs[3]*px*px + 2*coeffs[2]*px + coeffs[1];
>           epsi = psi - atan(derivative);

### Reference Velocity amnd Tuning
A reference velocity must be chosen and the cost function tuned accordingly. The goal is to SAFELY drive on the road at the fastest speed possible. The tuned cost function allows the car to SAFELY complete the track under all resolution and graphics quality settings.

Reference velocity

>    const double ref_v = 80 * 1600 / 3600;    // convert from mph to m/s

Cost function

>           fg[0] = 0;
>
>           // Cost from reference state
>           for (size_t i=0; i<N; ++i) {
>             fg[0] += CppAD::pow(vars[cte_start + i] - ref_cte, 2);          // penalise cte
>             fg[0] += 250 * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);        // penalise epsi
>             fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
>           }
>           // Cost from actuators - minimise the use of actuators
>           for (size_t i=0; i<N-1; ++i) {
>             fg[0] += CppAD::pow(vars[delta_start + i], 2);
>             fg[0] += CppAD::pow(vars[a_start + i], 2);
>           }
>           // Cost from actuators - minimise the use of extreme turns/accelerations
>           // (ie minimise the value gap between sequential actuations)
>           for (size_t i=0; i<N-2; ++i) {
>             fg[0] += 150 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);      // smoothing steer angle
>             fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
>           }
