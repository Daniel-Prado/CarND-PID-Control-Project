# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program
Daniel Prado Rodriguez - February Cohort
---

## Reflections

PID Controller is one of the most common control loop mechanism. A PID controller calculates an error value (cross-track error in this project) every iteration, as a difference between the desired target value and a measured one, and applies a correction input based on the (P)roportional, (I)ntegral and (D)erivative terms which give name to the controller.

In this project the cross track error is given by the Car Simulator, as the difference between the position of the car respective to the middle of the road, which is the desired target value. And the input correction is the steering value that the PID applies every iteration to follow the desired path.

The full formula of any PID controller, and as it is implemented in `src/pid.cpp` is as follows:
```
steer(cte): -Kp * cte - Kd * (cte - previous_cte) - Ki * total_cte;
```
where `Kp`, `Kd` and `Ki` are the proportional, derivative and integral components of the controller.

The proportional part `Kp` is responsible for the sensitivity of the controllers' response, and is directly proportional to the instantanious cross track error. The higher it is, the strongest (and fastest) it will react to the error. That implies that if we make this component very high (very sensitive) the controller will quickly start to oscilate and eventually overshoot out of control.

The derivative part `Kd` helps the PID controller to manage the overshoot caused by the `Kp`. It is applied to the difference between the current CTE and the previous one, so, if the error is decreasing, it will 'soften' the response, preventing the controller from oscilating.

The integral part `Ki` compensates for any constant bias the controlled object might have by adjusting the output to a sum of all cte's observed in the lifecycle of the controller (integral value).

## How I adjusted the PID values
The project rubric gives freedom on the technique used to tune the PID parameters. I chose to do it manually, at least in a first instance, because I wanted to truly understand how the controller works, and how tunning up and down the parameters affect the controller. To manually tune a PID is quite an 'art' as there is not a systematic approach that will give the optimal solution in every case, which makes the case more interesting. Finally, a first rough manual tunning of the parameters can be used to later fine-tune them using an automated algorithm.
Note that in this implementation I kept the throttle input fixed to 30mph.
So, this is the approach I followed based on recommendations on the course forums:
1. First, I set the Kp, Ki and Kd values to zero, disabling them.
2. Then I slightly increase the Kp component and run the simulator for 500 iterations. I make subjective observation, and objective measurements of the total error and average error every 100 iterations. Example:
```
LEG 1: AVERAGE ERROR OVER LAST 100 SAMPLES:0.725165
LEG 2: AVERAGE ERROR OVER LAST 100 SAMPLES:0.433132
LEG 3: AVERAGE ERROR OVER LAST 100 SAMPLES:0.206566
LEG 4: AVERAGE ERROR OVER LAST 100 SAMPLES:0.354679
LEG 5: AVERAGE ERROR OVER LAST 100 SAMPLES:0.449749
ACCUMULATED ERROR AFTER 500 SAMPLES: 218.324
```
As an example, I also show the results graphically, for Kp values between 0.1 and 1.0 in 0.1 increases:


3. I continue increasing Kp as long as the total error over 500 samples continues decreasing, also observing that there are not 'peak' errors along the 100 samples checkpoints. Visually, this is until the car starts to overshoot.
4. I then keep Kp fixed, and increase Kd until the overshoot is minimized. Here I need more data so I track the controller over 2000 samples. I chose a Kp value that gives a total error minimum (at least a local minimum)
```
LEG 1: AVERAGE ERROR OVER LAST 100 SAMPLES:0.586025
LEG 2: AVERAGE ERROR OVER LAST 100 SAMPLES:0.05502
LEG 3: AVERAGE ERROR OVER LAST 100 SAMPLES:0.064508
LEG 4: AVERAGE ERROR OVER LAST 100 SAMPLES:0.068676
LEG 5: AVERAGE ERROR OVER LAST 100 SAMPLES:0.070526
LEG 6: AVERAGE ERROR OVER LAST 100 SAMPLES:0.083608
LEG 7: AVERAGE ERROR OVER LAST 100 SAMPLES:0.155983
LEG 8: AVERAGE ERROR OVER LAST 100 SAMPLES:0.151906
LEG 9: AVERAGE ERROR OVER LAST 100 SAMPLES:0.244531
LEG 10: AVERAGE ERROR OVER LAST 100 SAMPLES:0.194848
LEG 11: AVERAGE ERROR OVER LAST 100 SAMPLES:0.092065
LEG 12: AVERAGE ERROR OVER LAST 100 SAMPLES:0.086022
LEG 13: AVERAGE ERROR OVER LAST 100 SAMPLES:0.108518
LEG 14: AVERAGE ERROR OVER LAST 100 SAMPLES:0.065502
LEG 15: AVERAGE ERROR OVER LAST 100 SAMPLES:0.344392
LEG 16: AVERAGE ERROR OVER LAST 100 SAMPLES:0.282896
LEG 17: AVERAGE ERROR OVER LAST 100 SAMPLES:0.344072
LEG 18: AVERAGE ERROR OVER LAST 100 SAMPLES:0.093622
LEG 19: AVERAGE ERROR OVER LAST 100 SAMPLES:0.102751
LEG 20: AVERAGE ERROR OVER LAST 100 SAMPLES:0.468733
ACCUMULATED ERROR AFTER 2000 SAMPLES: 370.42
```
5. With Kp and Kd fixed, I finally test slight adjustments to the integral component Ki. The Ki is very sensitive so the values need to be very small. On the other hand there would be no reason to believe there is a 'bias' in the simulator... but the fact that the circuit is driven counter-clock wise with all the turns being to the left, may induce it. Again, I apply the same numerical technique to evaluate the result, and this time I ensure the car drives the full circuit (over 3000 iterations aprox.)

Following this manual pseudo-algorithm I found to work best the following values:
* Kp: 5.25
* Ki: 0.0035
* Kd: 7.0

Below you can see a video of the simulator with the controller tuned with these parameters:

[![Project 4 Output](https://img.youtube.com/vi/tvfMtfEe4fk/0.jpg)](https://youtu.be/tvfMtfEe4fk)




---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
