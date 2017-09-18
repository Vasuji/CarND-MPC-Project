
## The Model

>Student describes their model in detail. This includes the state, actuators and update equations.

state: `[px, py, psi, v, cte, epsi]`

actuators: `[steering angle, throttle]`

update equations:

```
- x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
- y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
- psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
- v_[t+1] = v[t] + a[t] * dt
- cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
- epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

cost function:

```
 fg[0] = 0;
    
    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += 4*CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 4*CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    
    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 5*CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 40*CppAD::pow(vars[a_start + t], 2);
    }
    
    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 500*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```





## Timestep Length and Elapsed Duration (N & dt)

>Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

- In this project, the latency is set to 100ms. So, I set dt as 0.1s.
- I set T as 1.2s because the surrounding enviroment change quickly.
- As a result, I set N as 12.







## Polynomial Fitting and MPC Preprocessing & Model Predictive Control with Latency

>A polynomial is fitted to waypoints.
>If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.
>The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

preprocess waypoint:

- all of the landmark points are converted into the car's coordinate system.
```
vector<double> transform(double mapx, double mapy, double vehx, double vehy, double psi) {
  vector<double> ret;
  double dx = mapx - vehx;
  double dy = mapy - vehy;
  double r = sqrt(dx*dx + dy*dy);
  double angle2 = atan2(mapy-vehy, mapx - vehx);
  double angle = angle2-psi;
  ret.push_back(r*cos(angle));
  ret.push_back(r*sin(angle));
  return ret;
}
```




preprocess vehicle state:

- latency is taken into account (predict position 100ms after)

I used 3rd order of polynomial fit.






## The vehicle must successfully drive a lap around the track.

https://......








