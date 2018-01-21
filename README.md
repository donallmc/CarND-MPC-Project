# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Rubric Points

The general goal of this project is to produce a Model Predictive Control program that is capable of driving
a virtual car around the Udacity self-driving car simulator. The car should complete a full lap without its
tyres leaving the driveable portion of the track or perform any manoeuvre that would be considered "unsafe"
by any humans in the vehicle.

### The Model
The project employs a kinematic model which is a simplified representation of a car on a road, ignoring
complex features such as the interaction between the tyres and the road, drift, etc.

The model state is defined as ```[x, y, psi, v, cte, psi]``` where
- `x` and `y` correspond to the vehicle's position;
- `psi` is the vehicle's heading;
- `v` is the vehicle's velocity;
- `cte` is the vehicle's crosstrack error; and
- `epsi` is the vehicle's heading error.

These values are computable by the following set of equations, as provided in the course materials:
```
(1) x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
(2) y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
(3) psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
(4) v_[t+1] = v[t] + a[t] * dt
(5) cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
(6) epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt 
```
where:
- `t` represents a timestamp; 'dt' corresponds to the elapsed time; and
- Lf is a constant representing the distance between a car's centre of mass and its front wheels

The model outputs two actuators, `a` (throttle) and `delta` (steering angle) which are then used to drive the simulator.

### Cost Minimization
I used the standard cost function as provided in the quiz code, which sought to minimize the crosstrack error, heading error
and velocity delta, as well as penalizing actuator use and large changes between consecutive actuator uses.

Through trial and error, I discovered that applying a significant weight to the sequential actuations cost was the most
important tweak as it corrected wild over-steering in the model. I also applied a mild weight to the actuator use function
which was useful to modulate acceleration.

### Target Velocity
I initially set the target velocity to 40 but as I refined the model I experimented with pushing the velocity higher.
Through tweaking the cost function weighting, I experimented with a target velocity of 80; after the cost function waiting
this effectively resulted in a speed of around 65, but this produced a couple of "questionable" turns,
during which the car didn't leave the road but certainly would have caused any human passengers to be nervous! The submitted
code has a target velocity of 50, resulting in a fairly consistent speed of 40, which is a little modest but which produces
a perfectly safe lap around the track.

### Timestep Length and Elapsed Duration (N & dt)
The timestep and elapsed duration were initially set to the values provided in the sample code (25 and 0.05 respectively).
With some experimentation, it became apparent that 25 was an excessively large number and it caused failures in parts of
the track that involved multiple turns within a relatively short distance. I speculate that looking too far into the future
produced a line that couldn't be well-modelled by a 3rd-degree polynomial and this had a detrimental effect on accuracy.
Reducing the value of `N` to 1 produced even worse results so I set it to 15 and found that the model performed well in
the simulator. I kept the value of 0.05 for `dt` as I was already getting adequate performance and increasing it significantly
would likely result in problems due to latency.


### Latency
Latency is simulated by having the thread sleep for 100ms between computing values and sending them to the server.

---

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
