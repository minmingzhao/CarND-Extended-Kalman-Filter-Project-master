# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program: My 6th Self Driving Car ND Project, 1st Project at Term 2. 
The main goal of this project is to apply (extended) kalman filter from Lidar and Radar sensors of a self driving car using C++ language. 

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
  - Specifically, I followed instruction at [this link](http://tudat.tudelft.nl/projects/tudat/wiki/Install_on_Mac_OS_X) using dmg file for CMake. 
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * In case you have same issue like me in Mac, Mac seems to have GNU 3.81 Make under Commandline Tools, even latest Xcode does not contain newer Make version per my experience and [this stateoverflow poster - "updating-make-version-4-1-on-mac"](http://stackoverflow.com/questions/43175529/updating-make-version-4-1-on-mac). So step to update Make yourself is per the link above using homebrew: `brew install make --with-default-names`. `export PATH="/usr/local/bin:$PATH"` to update the correct newer location of Make. 
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * [InstallingGccMac](http://cs.millersville.edu/~gzoppetti/InstallingGccMac.html)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Contents of this repo
`src` a directory with the cpp codes:
* `main.cpp`- reads in data, calls a function to run the Kalman filter, calls a function to calculate RMSE
* `FusionEKF.cpp`- reads in data, calls a function to run the Kalman filter, calls a function to calculate RMSE
* `kalman_filter.cpp`- reads in data, calls a function to run the Kalman filter, calls a function to calculate RMSE
* `tools.cpp`- function to calculate RMSE and the Jacobian matrix

`data` folder:
* `obj_pose-laser-radar-synthetic-input.txt`: data file containing Lidar and Radar data. The columns are
  * For a row containing radar data, the columns are: sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth
  * For a row containing lidar data, the columns are: sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.

`results` a directory containing output and log files, as well as ekf visualization plot. 

`Docs` a directory with data file structure description

`Visualization` a directory with python script to visualize the ekf C++ output file. 

## Results
[ekf_visualization.png]: ./results/ekf_visualization.png
![alt text][ekf_visualization.png]

Accuracy (RMSE): [0.0972256, 0.0853761, 0.450855, 0.439588]. And it is less than the requirement. 
Project Requirement: [0.11, 0.11, 0.52, 0.52]

## Lesson learnt
* When converting cartesian coordinates to polar coordinates, we should use atan2(y,x) instead of atan(y/x) as the later could only do first and fourth quadrant (π/2 ~ π/2) while atan2 returns （-π ~ π）, covering all four quadrants. 
* When calculating phi in ｀y = z - h(x)｀ for radar measurements, the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi. The Kalman filter is expecting small angle values between the range -pi and pi. So when working in radians, we should add 2π or subtract 2π until the angle is within the desired range.

## How to run the code
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
4. Run it: `./ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt obj_output.txt > obj_input.log`. 

To have a detailed description of the project requirements, please see [this repo](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project). 


