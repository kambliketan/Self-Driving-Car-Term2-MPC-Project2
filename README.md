# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Hyperparameter Tuning

- We can tune following hyper parameters:
  1. N - the number of timesteps 
  2. dt - the time between actuations
  3. importance weights - to be applied to each type of cost.

- I started with N = 10 and dt = 100 ms as that is the latecy. 
- With all importance weights set to 1, I saw that the vehicle makes forward progress but tries very hard to correct it's trajectory with drastic steering movements between consecutive steps. And then soon falls off trajectory.
- Thus I tried increasing weigh to be applied to cost associated with drastic steering angle changes (see `weight_steer_cost`). This seemed to help a lot. I incrementally increased it from 10 to 100. At which point the vehicle could complete the trajectory without any issues.
- With reference velocity set to 40, with weight set to 1, I saw velocity changing from 35 - 42 so did not see need to tune the weight.

## Description

Kinematic model (see `main.cpp` lines `142` to `147`)

- ignore gravity and tire forces
- dynamic models are more elaborate and hence complex to implement
- model = state + actuator inputs
- defines how future state changes over time based on previous state and current actuator inputs

- x2 = x1 + v*cos(psi)*dt
- y2 = y1 + v*sin(psi)*dt
- psi2 = psi1 + (v/Lf)*steering_angle*dt
- v2 = v1 + acceleration * dt

- Lf measures the distance between the center of mass of the vehicle and it's front axle. The larger the vehicle, the slower the turn rate.
- If you've driven a vehicle you're well aware at higher speeds you turn quicker than at lower speeds. This is why vv is the included in the update.

State

- x, y, psi, velocity

Actuator commands

- steering_angle and acceleration
- acceleration is [-1, 1]

Fit

- Reference trajectory that the path planning system sends to Controller is typically a 3rd order polynomial.
- we will need to fit 3rd order polynomial to waypoints(x, y)

Cost (see `MPC.cpp` lines `60` to `79`)

- difference between reference trajectory (where you want vehivle to go) and actual vehicle path (prediction based on model and current state i.e where you think vehicle will actually go)
- [cte, e_psi] i.e cross_track_error and orientation_error can be added to state itself to track it over time

- cte2 = cte1 + v*cos(e_psi1)*dt
- e_psi2 = e_psi1 + (v/Lf)*steering_angle*dt

- Just cte and e_psi is not enough because then the vehicle might decide to stop or consecutive actuator inputs might differ drastically. Both of which is not practical. So we need to enhance the cost function.
- penalize for not maintaining reference velocity: cost += pow(v - 35, 2)
- another option is to add eucledean distance between vehicle and destination to the cost so that the vehicle keeps going.

- penalize drastic changes in actuator inputs: steering angle or accelaration so that the changes get smoothed out
- penalize just the magnitude: cost += pow(steering_angle, 2)
- penalize rate of change: cost += pow(steering_angle2, 2) - pow(steering_angle1, 2)

Model Predictive Control (see `MPC.cpp` class `MPC`)

- Represent the task of following a trajectory as an optimization problem
- involves simulating different actuator inputs, predict resulting trajectory and selecting trajectory with minimum cost

- initialize with current state and referrence trajectory we want to follow
-- optimize actuator inputs to each step in time in order to minimize cost of predicted trajectory
-- i.e. constantly calculate inputs over future horizon

- N is the number of timesteps in the horizon. dt is how much time elapses between actuations
- prediction horizon T = N * dt
- Thus the Control input vector that the MPC tries to optimize looks like this: [steering_angle1, acceleration1, steering_angle2, acceleration2, ... steering_angle_N-1, acceleration_N-1]

- Keep dt to minimum to avoid discretization error

- Putting it all together, the MPC involves-
1. solver: defining hyperparameters: N, dt, T.
2. solver: defining model
3. solver: defining contraints
4. solver: defining cost function
5. put the initial state through the solver, which returns control inputs that minimizes the cost function.
  - Fit polynomial to waypoints
  - calculate initial cte and e_psi
  - define components of cost function.
  - define constraints
6. apply first control input to the vehicle and then go to step 5.

- MPC can deal with latency much more effectively, by explicitly taking it into account, than a PID controller.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
