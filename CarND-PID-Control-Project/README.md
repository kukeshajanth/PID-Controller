# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Introduction

The purpose of this project was to "build a PID controller and tune the PID hyperparameters by applying the general processing flow as described in the lessons," and to "test your solution on the simulator!" The simulator provides cross-track error (CTE), speed, and steering angle data via local websocket. The PID (proportional/integral/differential) controller must respond with steering and throttle commands to drive the car reliably around the simulator track.

## Rubric Discussion Points

Describe the effect each of the P, I, D components had in your implementation.*

- The P, or "proportional", component had the most directly observable effect on the car's behavior. It causes the car to steer proportional (and opposite) to the car's distance from the lane center (which is the CTE) - if the car is far to the right it steers hard to the left, if it's slightly to the left it steers slightly to the right.

- The D, or "differential", component counteracts the P component's tendency to ring and overshoot the center line. A properly tuned D parameter will cause the car to approach the center line smoothly without ringing.

- The I, or "integral", component counteracts a bias in the CTE which prevents the P-D controller from reaching the center line. This bias can take several forms, such as a steering drift (as in the Control unit lessons), but I believe that in this particular implementation the I component particularly serves to reduce the CTE around curves.


## Tuning

The most important part of the project is to tune the hyperparameters. This can be done by different methods such as manual tuning, Zieglor-Nichols tuning, SGD, Twiddle. I have done manual tuning. Manual tuning is hard but, the process of tuning help us better understand every single effect of PID parameters. The following table summerizes the effect of each parameter on the system.


| Parameter  | Rise Time   | Overshoot  | Settling Time   | Steadystate error  |
|---|---|---|---|---|
| Kp  | Decrease  | Increase  | Small change  | Decrease  |
| Ki  | Decrease  | Increase  | Increase  | Decrease  |
| Kd  | Small change  | Decrease  | Decrease  | No change  |


The following approach is a best to tune manually:

* Set all gains to zero.
* Increase the P gain until the response to a disturbance is steady oscillation.
* Increase the D gain until the the oscillations go away (i.e. it's critically damped).
* Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
* Set P and D to the last stable values.
* Increase the I gain until it brings you to the setpoint with the number of oscillations desired.

I started initially with (0.5,0.1,0.5) for the steering control and (0.2,0.0,0.2) for speed control. The final parameters for my controllers are: 

|   | Steering  | Speed  |
|---|---|---|
| Kp  |  0.2|  0.12 |
| Ki  | 0.0  |  0.001 |
| Kd  | 1.0  |  0.0 |

My implementation was able to acheive a maximum speed of 35 mph without any oscillations. For speeds above 40 mph, the vehicle behaves aggresively. Below video shows the output for 30 mph.

<img src="30m.gif" width="480" alt="Combined Image" />


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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.







