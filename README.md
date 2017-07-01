**Extended Kalman Filter Project 1**
Self-Driving Car Engineer Nanodegree Program
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

---
##Purpose
The purpose of this project is to utilize a kalman filter to estimate the state of a moving object with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric, .11, .11, .52, .52. 

##Setup
Running the code and finding the RMSE values requires use of the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).  I used the Windows simulator build, though the code should be platform agnostic, and perform similarly on any host OS.

This repository includes two files to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) on either osx or linux, which is required for the simulator to communicate with the project code.  Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

The main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator is as follows.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

"estimate_x" <= kalman filter estimated position x
"estimate_y" <= kalman filter estimated position y
"rmse_x"
"rmse_y"
"rmse_vx"
["rmse_vy"]

##Simulator Output
[image1]: ./EFKProjectOutput2.png "Successfull output"
[image2]: ./EFKProjectOutput3.png "Angle Problem"
![alt text][image1]
This image shows the completed run of the Kalman Filter against the simulator, showing the complete figure 8 shape.

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Project Steps

The project purpose is to take in radar and lidar data, handle the two appropriately, and use them to build an internal prediction of the location of a movning object by using the kalman filter method covered in Term 2 of Udacity's Self Driving Car nanodegree.

Starter code was provided, with three files incomplete: kalman_filter.cpp, FustionEKF.cpp, and tools.cpp.  The missing code location in each file was marked with a TO DO comment, but the code itself is left up to the student to complete.  Much of the code had been built peice by peice during the prior coding quizes, so some of the work was simply moving that code into place.  For example tools.cpp required two functions be completed: CalculateJacobian() and CalculateRMSE().  Both of these were effectively already done by the time this project was started, so simple moving that quiz code into the file was sufficient to complete tools.cpp.

Other areas of the code, namely kalman_filter.cpp's UpdateEKF() function required more work.  While the forumlas for the extended version of the update function were displayed during the lectures, translating those formulas into C++ code proved difficult.  Other areas of the code required some refactoring or more robust error checking than the original quiz versions of the same functions.

## Ruberic

The project Ruberic defines what a passing project should look like.  I will review each of the Rubric points here individually.

###Your code should compile: Code must compile without errors with cmake and make.
The code compiles without error using cmake and make.

###px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] for Simulator Dataset 1
RMSE values of 0.0973, 0.0855, 0.4513, 0.4399 were observed during testing, below the threshold target.

###Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.
The code uses the same struture as the lesson code, initializing the internal state, predicting positions, updating internal state based on new measurements, handling both RADAR and laser input types, and repeating that process so long as data is received..

###Your Kalman Filter algorithm handles the first measurements appropriately.
When accepting an initial measurement of type laser, the kalman filter is initialized with those measurements directly.  When the initial measurement is of type RADAR, the kalman filter is initialized with a modified version of the measurements, translating the RADAR datum's distance and position into x and y coordinates to match the format of the laser data.

The code also attempts to catch any 0 or close to 0 values prior to performing divisions, so that div by 0 errors can be avoided.

###Your Kalman Filter algorithm first predicts then updates.
Following the structure of earlier lesson code, the algorithm predicts positions first, then updates the state data with the new measurement information.

###Your Kalman Filter can handle radar and lidar measurements.
After the initialization, laser and RADAR measurements are both handled, with slight differences depending on which data type is received.  Laser data is handled just as it was for the non"extended" Kalman Filter, whereas the RADAR data is handled a bit differently.  Because the RADAR data comes in using polar coordinates, the Kalman Filter update step (method updateEKF()) translates the data to cartesian, altering the H matrix into a modified form Hj, and Using Fj in place of F (u is considered to be 0).  

###Your algorithm should avoid unnecessary calculations.
In a few places, a calculation is needed more than once.  Rather than wasting CPU cycles recalculated the result each time it is needed, a temporary holding variable is used.  The value is calculated once, then referenced multiple times.  An example of this is in kalmann_filter.cpp, where the transpose of matrix H_ is needed 2 times in the subsequent lines.  Rather than call H_.transpose() twice, I have calculated the value once and stored it in variable Ht, referencing Ht both times.  Similarly, then the size of ekf_.x_ is needed to build an Identity matrix of the same size, I calculate the size of x_ once, store it in xsize, and reference that to build the identity matrix I on the next line.

##Other thoughts.
On the initial test run, I encountered a Segmentation Fault error which killed the ExtendedKF program.  After some debugging, it turned out that the ekf_ object instance's Q_ matrix was not being initialized in the same way as it had been in the lesson code.  By adding a declaration line right before assigning a value to that matrix, the error was fixed.

Once the segmentation fault was resolved, the Simulator output green triangles along the path of the car, denoting the predicted object location.  At one point, when the car passed below 0 on the y axis while travelling its figure 8 path, the predictions (green triangles) veered off widely, only re-converging on the car's path a number of frames later.  After investigation, it turned out that the y calculation step VectorXd y = z - h; was failing when the object was close to 0 on the y scale.  In that case, the value for the second value in Y was multiple times what we needed for it to make sense: something between -pi and pi.  To normalize this value to the range [-pi,pi], I added a line which used the atan2() function to normalize it.
![alt text][image2]

After resolving those two issues, the predictions appeared to follow the main path, and the RMSE was within acceptable thresholds.
