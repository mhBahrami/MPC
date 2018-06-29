# Model Predictive Controller (MPC)

In this project, I implemented Model Predictive Control to drive the car around the track. This time however you're not given the cross track error, you'll have to calculate that yourself! Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

---

## The Model

The vehicle model used in this project is a kinematic bicycle model. It neglects all dynamical effects such as inertia, friction and torque. The model takes changes of heading direction into account and is thus non-linear. The model used consists of the following equations

```c++
// Recall the equations for the model:
x_[t]    = x[t-1]    + v[t-1] * cos(psi[t-1]) * dt
y_[t]    = y[t-1]    + v[t-1] * sin(psi[t-1]) * dt
psi_[t]  = psi[t-1]  - v[t-1] / Lf * delta[t-1] * dt
v_[t]    = v[t-1]    + a[t-1] * dt
cte[t]   = f(x[t-1]) - y[t-1]      + v[t-1] * sin(epsi[t-1]) * dt
epsi[t]  = psi[t]    - psides[t-1] - v[t-1] * delta[t-1] / Lf * dt
```

which,

- **`x, y` :** the position of the car
- **`psi`:** the heading direction
- **`v`:** the car velocity
- **`cte`:** the cross-tracking error
- **`epsi`:** the orientation error
- **`Lf`:** the distance between the center of mass of the vehicle and the front wheels and affects the maneuverability.

>The model can be found in the class [FG_eval (line 139)](https://github.com/mhBahrami/MPC/blob/master/src/MPC.cpp#L139) .

### The State

The state contains four parameters as follows:

- **`x, y`:** car position
- **`psi`:** the car orientation
- **`v`:** the car velocity

### The Actuators

We can control the system (here a self driving car) by its actuators. For our system there are two actuators:

- **`delta`:** the steering angle
- **`a`:** the car acceleration which is known as throttle and/or brake pedals.

### How the model works?

The steps are as follows:

1. Pass the current state as the INITIAL STATE to the MPC.
2. Call the optimization solver. Given the INITIAL STATE, the solver (`Ipopt`) will minimize the cost function and return the relevant vector of control inputs. 
3. Apply the obtained control input to the car.
4. Repeat the previous steps again!

## Tuning The Parameters

There are two tuning parameters **`N`** (the number of time steps) and **`dt`** (the length of each time step). 

You can see the different `N` and `dt` that I chose in the below table.

|  #   | `N`  | `dt`  | Description                                                  |
| :--: | :--: | :---: | ------------------------------------------------------------ |
|  1   | `10` | `0.1` | The green path would often deviate to the right or left near the end, so I tried increasing `N` and then the model will fit more of the upcoming path and would be penalized more if it curved off erratically after `10` steps. |
|  2   | `15` | `0.1` | Increasing the `N`  improves the fit and made the car drive smoother. |
|  3   | `10` | `0.2` | Increasing the `dt`  makes the model response time larger and it causes the car crashes often. Because the model re-evaluate the model less frequently. |
|  4   | `20` | `0.1` | Increasing the `N`  more than `15` will reduce the stability of the car in the path. |

So, I chose `N` and `dt` equal to [`15`](https://github.com/mhBahrami/MPC/blob/master/src/MPC.cpp#L12) and [`0.1`](https://github.com/mhBahrami/MPC/blob/master/src/MPC.cpp#L13), respectively.

## The Latency

If you don't consider the latency, the car will do based on what it should have done `t_latency` ago. It means the car will change its position (steering angle) and its speed (throttle) too late! For instance it might not start steering let say to the left when the path is turning to the left and it leads the car to go off the track. The higher the speed, the greater the damage could be.

For our car in the simulator the latency time is `100 ms`.

### Integrating the latency to the implementation

It's easy! We should use our model to predict the state `100 ms` (the latency time) ahead of time and then feed that `state` to the MPC solver.

> You can find it in the `main.cpp` (from [line 144](https://github.com/mhBahrami/MPC/blob/master/src/main.cpp#L144) to [line 158](https://github.com/mhBahrami/MPC/blob/master/src/main.cpp#L158)). 

**NOTE: Predicting the state `100 ms` ahead gives a worse outcome than predicting the state with no latency time!** 

## The End Result (Video)

I recorded the end result and you can [watch it on YouTube](https://youtu.be/cWWs9NkSMBo).

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

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## License

[MIT License](https://github.com/mhBahrami/MPC/blob/master/LICENSE).