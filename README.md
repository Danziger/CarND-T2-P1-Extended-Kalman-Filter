CarND · T2 · P1 · Extended Kalman Filter Project
================================================

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<!-- <img src="examples/images/final.jpg" width="512" alt="Extended Kalman Filter Visualization." /> -->


Project Overview
----------------

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the [project rubric](https://review.udacity.com/#!/rubrics/748/view): `px = .11, py = .11, vx = 0.52, vy = 0.52` 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

The files that need to be completed to accomplish the project are:

- `src/FusionEKF.h`
- `src/FusionEKF.cpp`
- `src/kalman_filter.h`
- `src/kalman_filter.cpp`
- `src/tools.h`
- `src/tools.cpp`

The program `src/main.cpp` has already been filled out, but feel free to modify it. Also, feel free to change the file structure/architecture that has been provided in the started code if you wish.


Dependencies
------------

- [`cmake >= 3.5`](https://cmake.org/install/)
- `make >= 4.1` (Linux / [Mac](https://developer.apple.com/xcode/features/)), [`3.81` (Windows)](http://gnuwin32.sourceforge.net/packages/make.htm)
- `gcc/g++ >= 5.4` (Linux / [Mac](https://developer.apple.com/xcode/features/)), [`MinGW` (Windows)](http://www.mingw.org/)


Installation
------------

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets):

- `install-mac.sh` for Mac.
- `install-ubuntu`for either Linux or [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/).

For Windows, Docker or VMware coulso also be used as explained in the [course lectures](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77).

Details about enviroment setup can also be found there.


Build
-----

Once the install is complete, the main program can be built and run by doing the following from the project top directory:

1. Create a build directory and navigate to it: `mkdir build` && `cd build`
2. Compile the project: `cmake .. && make`
3. Run it: `./ExtendedKF`


Data Flow
---------

1. The measuremennt processor/MATLAB simulator is generating the input file `data/obj_pose-laser-radar-synthetic-ukf-input.txt` with the following format:

    **For laser:**
  
    | sensor | meas_px | meas_py | timestamp | gt_px | gt_py | gt_vx | gt_vy |
    |--------|---------|---------|-----------|-------|-------|-------|-------|
    | L | 8.45 | 0.25 | 1477010443349642 | 8.45 | 0.25 | -3.00027 | 0 |
        
    **For radar:**
  
    | sensor | meas_rho | meas_phi | meas_rho_dot | timestamp | gt_px | gt_py | gt_vx | gt_vy |
    |--------|----------|----------|--------------|-----------|-------|-------|-------|-------|
    | R | 8.60363 | 0.0290616 | -2.99903 | 1477010443399637 | 8.6 | 0.25 | -3.00029 | 0 |
    
2. The simulator reads all the lines and generates measurement structures that are sent to `main.cpp` using `uWebSocketIO` (port `4567`).

3. The `FusionEKF::MeasurementProcessor()` is called with individual measurements (one by one), which will update the Kalman filter state.

4. `main.cpp` accesses the internal `FusionEKF` and `KalmanFilter` instances' internal states and read the current estimated `px`, `py`, `vx` and `vy` and uses them to calculate multiple `RMSE` (on for each).

5. `main.cpp` sends the following data using `uWebSocketIO` back to the simulator:

- `estimate_x = px`
- `estimate_y = py`
- `rmse_x = RMSE(px)`
- `rmse_y = RMSE(py)`
- `rmse_vx = RMSE(vx)`
- `rmse_vy = RMSE(vy)`


Generating Additional Data
--------------------------

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.
