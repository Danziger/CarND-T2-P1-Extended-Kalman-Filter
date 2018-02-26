CarND · T2 · P1 · Extended Kalman Filter Project
================================================

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<img src="output/images/004 - Simulator Rotated.png" alt="Extended Kalman Filter visualization on the simulator." />


Project Overview
----------------

In this project you will utilize an Extended Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining `RMSE` values that are lower that the tolerance outlined in the [project rubric](https://review.udacity.com/#!/rubrics/748/view): `px = 0.11, py = 0.11, vx = 0.52, vy = 0.52` 

To test it, [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases) need to be used. The latest version of `main.cpp` used to run this project without the simulator can be found [here](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project/blob/06cbc9967bc62592723eef99b8c8035e4a22ea7b/src/main.cpp).


Dependencies
------------

- [`cmake >= 3.5`](https://cmake.org/install/)
- `make >= 4.1` (Linux / [Mac](https://developer.apple.com/xcode/features/)), [`3.81` (Windows)](http://gnuwin32.sourceforge.net/packages/make.htm)
- `gcc/g++ >= 5.4` (Linux / [Mac](https://developer.apple.com/xcode/features/)), [`MinGW` (Windows)](http://www.mingw.org/)


Installation
------------

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets):

- `install-mac.sh` for Mac.
- `install-ubuntu`for either Linux or [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) (**please, make sure it is updated**).

For Windows, Docker or VMware coulso also be used as explained in the [course lectures](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77). Details about enviroment setup can also be found there.


Build
-----

Once the install is complete, the main program can be built and run by doing the following from the project top directory:

1. Create a build directory and navigate to it: `mkdir build && cd build`
2. Compile the project: `cmake .. && make`
3. Run it: `./ExtendedKF`


Relevant Changes
----------------

These are some of the more relevant changes I made to the started code or to the suggestions from the lessons:

**Code Style and Code Organization**

- Use of 4 spaces instead of 2 for indentation.
- Rename of files with classes to match its CamelCase name.
- Rename of `FusionEKF` to `EKFTracker`.
- Rename of `KalmanFilter` to `EKF`.
- Letter case-separated words to name variables and class methods.
- Make all class properties private and create methods to access and update them.

**Code Functionality**

- Move `Tools::calculateJacobian` to `EKF::calculateJacobian`.
- Add `EKFTracker::getCurrentState` and `EKF::getCurrentState` and updats `main.cpp` accordingly so that it can still access the Kalman Filter state after each simulator update.
- Remove all Kalman Filter matrixes from `EKFTracker` and add them to `EKF`, either as class properties or as local variables, depending if they can be reused or not.
- Update `EKF::predict` to accept the elapsed as a param and incorporate it to the matrixes `F` and `Q`.
- Update division by zero checks to use a saturation value instead of logging out and error.
- Create methods `EKF::initMatrixes`, `EKF::initState` and `EKF::initNoise`.
- Create methods `EKF::updateLaser` and `EKF::updateRadar`, which use the internal `R_laser_` and `R_radar_` matrixes, respectively, as well as `EKF::update` and `EKF::updateEKF`, which accept an `R` covariance matrix as a param to be used on the update.
- Add improved division-by-zero prevention using the macros defined in `EKF.cpp`.

Data Flow
---------

1. The measuremennt processor/MATLAB simulator is generating the input file `data/obj_pose-laser-radar-synthetic-ukf-input.txt` with the following format:

    **For laser:**
  
    | `sensor` | `meas_px` | `meas_py` | `timestamp` | `gt_px` | `gt_py` | `gt_vx` | `gt_vy` |
    |----------|-----------|-----------|-------------|---------|---------|---------|---------|
    | `L` | `8.45` | `0.25` | `1477010443349642` | `8.45` | `0.25` | `-3.00027` | `0` |
        
    **For radar:**
  
    | `sensor` | `meas_rho` | `meas_phi` | `meas_rho_dot` | `timestamp` | `gt_px` | `gt_py` | `gt_vx` | `gt_vy` |
    |----------|------------|------------|----------------|-------------|---------|---------|---------|---------|
    | `R` | `8.60363` | `0.0290616` | `-2.99903` | `1477010443399637` | `8.6` | `0.25` | `-3.00029` | `0` |
    
2. The simulator reads all the lines and generates measurement structures that are sent to `main.cpp` using `uWebSocketIO` (port `4567`).

3. `EKFTracker::processMeasurement()` is called with individual measurements (one by one), which will update the Kalman Filter state like so:

    **High-level overview of measurement processing**
    
    <img src="output/images/001 - Process Measurement.png" alt="Process measurement diagram." />

    **Kalman Filter and Extended Kalman Filter matrices equations**
    
    <img src="output/images/002 - KF and EKF equations.jpg" alt="KF and EKF equations." />


4. `main.cpp` gets the `EKF` instance state throguth `EKFTracker` (current estimated `px`, `py`, `vx` and `vy`) and uses it to calculate the `RMSE`.

5. `main.cpp` sends the following data using `uWebSocketIO` back to the simulator, which will plot them.

    - `estimate_x = px`
    - `estimate_y = py`
    - `rmse_x = RMSE(px)`
    - `rmse_y = RMSE(py)`
    - `rmse_vx = RMSE(vx)`
    - `rmse_vy = RMSE(vy)`


Results
-------

With the default values provided in the started project, the results obtained on the simulator are:

| Dataset  | Sensor   | `RMSE X`  | `RMSE Y`  | `RMSE VX`  | `RMSE VY` |
|----------|----------|-----------|-----------|------------|-----------|
| `1`      | `L + R`  | `0.0973`  | `0.0855`  | `0.4513`   | `0.4399`  |
| `2`      | `L + R`  | `0.0726`  | `0.0967`  | `0.4579`   | `0.4966`  |
| `1`      | `L`      | `0.1222`  | `0.0984`  | `0.5825`   | `0.4567`  |
| `2`      | `L`      | `0.0961`  | `0.1003`  | `0.5418`   | `0.4640`  |
| `1`      | `R`      | `10.9958` | `7.7916`  | `10.1094`  | `7.8036`  |
| `2`      | `R`      | `0.2244`  | `0.2954`  | `0.5870`   | `0.7338`  |

I have been playing around with the `noiseAX_` and `noiseAY_` values, and these are the resuls for the Dataset 1 when using both sensors' data:

| Noise AX / AY | `RMSE X` | `RMSE Y` | `RMSE VX` | `RMSE VY` |
|---------------|----------|----------|-----------|-----------|
| `10` | `0.0961` | `0.0846` | `0.4484` | `0.4330` |
| `11` | `0.0951` | `0.0840` | `0.4463` | `0.4274` |
| `12` | `0.0942` | `0.0836` | `0.4447` | `0.4226` |
| `20` | `0.0907` | `0.0834` | `0.4410` | `0.4039` |
| `24` | `0.0899` | `0.0841` | `0.4420` | `0.4012` |
| `32` | `0.0889` | `0.0856` | `0.4457` | `0.4013` |
| `40` | `0.0885` | `0.0872` | `0.4503` | `0.4053` |
| `1000` | `0.0995` | `0.1201` | `0.7518` | `0.9068` |

Increasing their value up to a certain point, probably around `20 - 24`, seems to help reduce the `RMSE`. Reducing them, however, had the opposite effect.

If we run again all the previous tests with `noiseAX_ = noiseAY_ = 24`, we get the following results:

| Dataset  | Sensor   | `RMSE X`  | `RMSE Y`  | `RMSE VX`  | `RMSE VY` |
|----------|----------|-----------|-----------|------------|-----------|
| `1`      | `L + R`  | `0.0889`  | `0.0856`  | `0.4457`   | `0.4013`  |
| `2`      | `L + R`  | `0.0735`  | `0.0869`  | `0.4497`   | `0.4582`  |
| `1`      | `L`      | `0.1185`  | `0.1012`  | `0.5979`   | `0.4623`  |
| `2`      | `L`      | `0.0967`  | `0.0981`  | `0.5450`   | `0.4698`  |
| `1`      | `R`      | `10.4272` | `7.2866`  | `11.6866`  | `9.6269`  |
| `2`      | `R`      | `0.2105`  | `0.2674`  | `0.5780`   | `0.6695`  |

We can see how, overall, we haven't improved too much, or at least we haven't done that consistently for all the cases, which might indicate we are overfitting the Dataset 1 with L and R sensor, which we were using while adjusting the noise values.


Generating Additional Data
--------------------------

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.


Interesting Resources
---------------------

- [Detailed explanations (55 videos) about Kalman Filters by Michel van Biezen @ iLectureOnline.](https://www.youtube.com/watch?v=CaCcOwJPytQ)

  
