#include "kalman_filter.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */
  
   x_ = F_ * x_;
   P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
  Eigen::VectorXd y;
  Eigen::MatrixXd S_;
  Eigen::MatrixXd K_;
  
  y = z - H_ * x_;
  CalculateMeasurements(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  
  // convert cartesian to polar coordinates
  float px, py, vx, vy;
  px = x_[0];
  py = x_[1];
  vx = x_[2];
  vy = x_[3];

  float rho, phi, rho_dot;
  rho = sqrt(px*px + py*py);
  phi = atan2(py, px);  

  if(rho > 0.0001) rho_dot = (px * vx + py * vy) / rho;
  else rho_dot = 0.2;

  VectorXd z_p = VectorXd(3);
  z_p << rho, phi, rho_dot;
  
  Eigen::VectorXd y;
  
  y = z - z_p;
  while(y(1) < -1 * M_PI) 
    y(1) += 2 * M_PI;
  while( y(1) > M_PI) 
    y(1) -= 2 * M_PI;
  
  CalculateMeasurements(y);
}

void KalmanFilter::CalculateMeasurements(const Eigen::VectorXd &y){
  Eigen::MatrixXd S_;
  Eigen::MatrixXd K_;
  S_ = H_ * P_ * H_.transpose()+ R_;
  K_ = P_ * H_.transpose() * S_.inverse();
  x_ = x_ + K_ * y;
  P_ = (MatrixXd::Identity(4, 4) - K_ * H_) * P_;
}
