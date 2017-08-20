# The Model
These are the basic model equations used.  x and y are the position of the
vehicle in vehicle coordinates.  psi is the orientation. v is the velocity.
cte is crosstrack error and epsi is the orientation error.

* x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
* y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
* psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
* v[t+1] = v[t] + a[t] * dt
* cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
* epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

# Timestep Length and Elapsed Duration (N & dt)
The final values used for N is 10 and dt is 0.1 seconds.  This gives a T value
of 1 second for the prediction horizon.  Increasing N made the computation time
become longer and created problems for completing the calculation in a
sufficient amount of time.  Smaller and larger values for dt seemed to cause
instability, but these could possibly have been compensated with tuning in
other areas.  However, the 0.1s dt provided a satisfactory result.


# Polynomial Fitting and MPC Preprocessing
In main.cpp line 100, the simulator waypoints are converted from global
coordinates to vehicle coordinates.  On line 132, the values are modified using
the model equations to compensate for latency, see next section.


# Model Predictive Control with Latency
The latency compensation is calculated starting with line 132 of main.cpp.  A
value of 0.12s is used with 100ms plus 20ms to help compensate for calculation
time.  This works at 50mph and also at higher speeds, but there are still the
occasional blip.  These did not cause the vehicle to completely lose control.

On MPC.cpp line 266, an alternative strategy of using an offset into vars for steering and throttle
values returned from MPC.Solve was attempted, but did not provide good results
in testing.  It feels like this should work, but there may be more that needs
to be done to fully account for other latency components.

On MPC.cpp line 242, the max_cpu_time option of the solver was set to 30ms.
This put an upper bound on the cpu time since as error started increasing the
solver would take longer causing a downward spiral in overall latency leading
to vehicle crashes.

