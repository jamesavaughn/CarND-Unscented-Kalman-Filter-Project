![GitHub Logo](/images/screenshot.png)
Format: ![Alt Text](url)

# Unscented Kalman Filter Project

This project utilizes an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

RMSE values are calculated to ensure filter consitency.  

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

## Installation
Clone the GitHub repository and run the build directory
* Step 1: '$ git clone`
* Step 2: `$ cmake`
* Step 3: `$ make`
* Step 4: `./ExtendedKF`
-- will return
Listening to port 4567
Connected!!
* Step 5: Run the simulator

## Project Rubric Results in the table below
Rubric Point | Result
------------ | ------------
Compiling - Code compiles using `cmake` and `make`. | YES
Accuracy - Algorithm runs against "obj_pose-laser-radar-synthetic-input.txt. px, py, vx, and vy RMSE should be less than or equal to the values [.09, .10, .40, .30] | Results:  [.0828 , .0004, 21.7 , .7081]
Correct Algorithm - follows genral processing flow | YES     
KF algorithem handles first measurements | YES
KF first predicts then updates | YES
KF handles radar and lidar | YES
Code Efficiency | YES


## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
