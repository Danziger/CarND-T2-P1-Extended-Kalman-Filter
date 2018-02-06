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





Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---




## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.
