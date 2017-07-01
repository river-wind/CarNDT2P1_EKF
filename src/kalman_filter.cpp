#include "kalman_filter.h"
#include <math.h> 
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  //CEL: KF Prediction step		
  x_=(F_*x_);
  P_=F_*P_*F_.transpose()+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  //CEL: update from standard KF
  VectorXd y=z-(H_*x_);
  // new state
  MatrixXd Ht=H_.transpose();
  MatrixXd S=H_*P_*Ht+R_;
  MatrixXd K=P_*Ht*S.inverse();
  x_=x_+(K*y);
  int xsize=x_.size();
  MatrixXd I=MatrixXd::Identity(xsize,xsize);
  P_=(I-(K*H_))*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */ 
  float rho=sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi=atan2(x_(1), x_(0));
  float rho_d;
  VectorXd h(3);

  if (rho<0.0001) {
    rho_d = 0;
    rho = 0.0001;
  } else {
    rho_d = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  }
//  VectorXd h(3);
  h << rho, phi, rho_d;
  VectorXd y=z-h;
  //normalize the resulting y angle
  y[1] = atan2(sin(y[1]),cos(y[1]));
  // new state
  MatrixXd Ht=H_.transpose();
  MatrixXd S=H_*P_*Ht+R_;
  MatrixXd K=P_*Ht*S.inverse();
  x_=x_+(K*y);
  int xsize=x_.size();
  MatrixXd I=MatrixXd::Identity(xsize,xsize);
  P_=(I-(K*H_))*P_;
}
