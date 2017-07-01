#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include "math.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

//cout<<"init1"<<endl;
  previous_timestamp_ = 0;
//cout<<"init2"<<endl;
  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  //Q_ = MatrixXd(4, 4);
//cout<<"init3"<<endl;
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;
//cout<<"init4"<<endl;
  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
//cout<<"init5"<<endl;
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  //CEL: set the measurement matrix H_
  //measurement matrix
  //kf_.H_ = MatrixXd(2, 4);
  //kf_.H_ << 1, 0, 0, 0,
  //	    0, 1, 0, 0;
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1; 
//cout<<"init6"<<endl;
  //the initial transition matrix F_
  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
  //CEL: set the transition matrix F_
  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
  	     0, 1, 0, 1,
	     0, 0, 1, 0,
	     0, 0, 0, 1;
//cout<<"init7"<<endl;
  //CEL: set the process and measurement noises
  //set the acceleration noise components
  //float noise_ax = 9.0;
  //float noise_ay = 9.0;
//  cout<<"constructed"<<endl;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

//cout<<"ProcM1"<<endl;
  /*****************************************************************************
   *  Initialization
  
 ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      //CEL: calculate the x,y position based on angle (phi) and range (rho)
      //CEL: px is adjacent to phi and might be calculated as rho * cos(phi)
      //CEL: py is opposed to phi and might be calculated as rho * sin(phi)
           //to remove: csc = cosign(measurement_pack.raw_measurements_[0])
      //cout<<"initializing for RADAR"<<endl;
      float px = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      float py = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
      //to remove:  py = sqrt(exp(px,2)+exp(measurement_pack.raw_measurements_[1],2))
      ekf_.x_  << px, py, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //cout<<"initializing for laser"<<endl;
      //CEL: get initial measurements and set ekf_.x_, as in lesson 5.12
      ekf_.x_ << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1], 0, 0;
    }
    //cout<<"ProcM2"<<endl;
    //CEL: create the initial covariance matrix (P? or should this initiate Q?)
    //state covariance matrix P
    //kf_.P_ = MatrixXd(4, 4);
    //kf_.P_ <<     1, 0, 0, 0,
//		  0, 1, 0, 0,
//		  0, 0, 1000, 0,
//		  0, 0, 0, 1000;
//cout<<"ProcM3"<<endl;
    //CEL: initialize the timestamp for measuring deltas
    previous_timestamp_ = measurement_pack.timestamp_;
//cout<<"ProcM4"<<endl;
    Tools tools;
//cout<<"ProcM5"<<endl;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
//cout<<"ProcM6"<<endl;
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  //CEL: using the time clacluations and F_ update logic from the lessons
  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
//cout<<"ProcM7"<<endl;	
  //1. Modify the F matrix so that the time is integrated
  ekf_.F_ << 1, 0, dt, 0,
	  0, 1, 0, dt,
	  0, 0, 1, 0,
	  0, 0, 0, 1;
//cout<<"ProcM8"<<endl;
  //CEL: set the process and measurement noises
  //set the acceleration noise components
  float noise_ax = 9.0;
  float noise_ay = 9.0;
//cout<<"ProcM9"<<endl;
  //2. Set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << ((std::pow(dt,4))/4)*noise_ax, 0, ((std::pow(dt,3))/2)*noise_ax, 0,
  		  0, ((std::pow(dt,4))/4)*noise_ay, 0, ((std::pow(dt,3))/2)*noise_ay,
		  ((std::pow(dt,3))/2)*noise_ax, 0, ((std::pow(dt,2)))*noise_ax, 0,
		  0, ((std::pow(dt,3))/2)*noise_ay, 0, ((std::pow(dt,2)))*noise_ay;

//cout<<"ProcM9.5"<<endl;
  //3. Call the Kalman Filter predict() function
  ekf_.Predict();
//cout<<"ProcM10"<<endl;
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // measurement in polar coordinates, need to something something Jacobian linearization maybe.  tools function! 
//cout<<"ProcM11"<<endl;
    ekf_.R_ = R_radar_;
//cout<<"ProcM12"<<endl;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
//cout<<"ProcM13"<<endl;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_); 
 } else {
    // Laser updates
    //CEL: measurements already as expected, no change from lesson code
//cout<<"ProcM14"<<endl;
    ekf_.R_ = R_laser_;
//cout<<"ProcM15"<<endl;
    ekf_.H_ = H_laser_;
//cout<<"ProcM16"<<endl;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
//cout<<"ProcM20"<<endl;
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
