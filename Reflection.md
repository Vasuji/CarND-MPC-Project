
## The Model

>Student describes their model in detail. This includes the state, actuators and update equations.

* state: The state comprises of 6 fields, which are:  `[px, py, psi, v, cte, epsi]`

* actuators: `[steering angle, throttle]`

The actuators of the system are acceleration and steering angle. These are the main outputs of the curve fitting using <code> IPOPT/CppAD </code> libraries which are used in the MPC.cpp. 

update equations:

```
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + cte_start + t] =
  cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[1 + epsi_start + t] =
  epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
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

- latency is taken into account (predict position 100ms after). 

I used 3rd order of polynomial fit.



## The vehicle must successfully drive a lap around the track.

https://......








