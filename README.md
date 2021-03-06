# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## The effect of the P, I, D component of the PID algorithm in their implementation

#### Effect of the P component

For reducing the "crosstrack error"(CTE), which is the lateral distance between the vehicle and the so-called reference trajectory, we could set the steering angle in proportion to what's known as . P stands for the proportional. When P is larger, the car will oscillate faster. When P is too small, the car will go out of the road.

#### Effect of the D component

When using P-controller, the car will move towards a trajectory more and more, but the car itself will still be oriented a little bit downwards, so it's forced to overshoot. This problem is  what's called "marginally stable". And the D-controller is used to solve this problem.

#### Effect of the I component

To solve the problem "systematic bias", which is  a mistake caused by mechanic, we use the I-controller.

## How to chose the final hyperparameters (P, I, D coefficients)

I try to use twiddle; But the hyperparameters always become negative and then the car go out of the road. What's wrong with my code?

Firstly, I tune the hyperparameters manually; Steps as following:

1. Set all gains to zero.

2. add P gain until the car can go through the first turn successfully;(0.04-0.05)

3. add D gain until the the oscillations go away;(1.2-2)

4. Repeat step 2 and 3 until increasing the D gain does not stop the oscillations(P: 0.11, D: 2.0);

5. Increase the I gain until the car can go through the road.(I : 0.001)

Secondly, I try to tune the hyperparameters by twiddle; I start twiddling from the best hyperparameters manually; but I can't let the car go through the road;
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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)



