# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Model
A Model Predictive Controller is used to change the acceleration and the steering angle, which are the actuators, along a path in
the simulation. The MPC assumes no tire forces, gravity or impact from mass. This class of MPCs are often referred to as bicycle
models. To mimic the travel time of actuators through the car, the model takes into account the latency. Next we will discuss the
mathematical model, the hyper parameters and the how we account for latency.

### Mathematical Model
Let the coordinates of the car at time $t$ be denoted by the pair $x_t, y_t$. The steering angle of the car is denoted as
$\psi_t$. The velocity of the car at time $t$ is denoted as $v_t$. The model has two actuators which are the steering angle
$\delta_t$ and the throttle $a_t$. The model includes the distance between its center of gravity and its front as $Lf$.

The dynamics of the car are defined as,
$$
     x_{t+1} = x_t + v_t \cdot cos(\psi_t) dt
$$

$$
     y_{t+1} = y_t + v_t \cdot sin(\psi_t) dt
$$
$$
     \psi_{t+1} = \psi_t + \frac{v_t}{Lf} \delta_t dt
$$
$$
     v_{t+1} = v_t + a_t dt
$$

Core to the MPC are the error metrics that we will use to ensure that the car drives along the planned path. We consider two error
metrics. The first error metric is the cross track error, $CTE_t$ and indicates how far the car deviates from the path in
location. The second error metric measures how much the angle deviates from the desired output and is denoted $e\psi_t$
$$
     CTE_{t+1} = f(x_t) - y_t + v_t \cdot sin(e\psi_t)  dt
$$
$$
     e\psi_{t+1} = \psi_t - psides_t + v_t \cdot \frac{delta_t}{Lf}  dt
$$

The model has a time horizon of $N = 10$ steps with $dt=0.1$. We chose these parameters through error and trial. Increasing the
horizon requires more calculation steps without necessarily increasing the accuracy, whereas reducing the discretization step $dt$
reduces the amount by which the MPC model is able to predict ahead.

### Solving the Model
To solve the model we set up the following cost function,
$$
J = \sum_{\tau = t + 1}^{t + N} w_{CTE}  CTE_\tau^2 + w_{e\psi}e\psi_tau^2 + w_{v} (v_\tau - v_{ref})^2 + w_\delta\delta_tau^2 +
w_aa_\tau^2 + w_\dot{\delta} (\delta_{\tau} - \delta_{\tau+1})^2 + w_\dot{a} (a_{\tau} - a_{\tau+1})^
$$
where the different $w$'s are the weights in the cost function that penalize certain factors more than others. Through trial and
error we found that for a reference speed $v_{ref}=40$ the $w_{CTE} = 5, w_{e\psi} = 5, w_v = 5, w_{\dot{\delta}}=600$.

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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
