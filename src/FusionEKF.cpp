#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  
  Eigen::MatrixXd F_ = MatrixXd(4, 4);
  Eigen::MatrixXd P_ = MatrixXd(4, 4);
  Eigen::MatrixXd Q_;
  
  P_ << 1, 0,   0,   0,
        0, 1,   0,   0,
        0, 0, 100,   0,
        0, 0,   0, 100;
    
  F_ << 1, 0, 0.1,   0,
        0, 1,   0, 0.1,
        0, 0,   1,   0,
        0, 0,   0,   1;
  Q_ = MatrixXd::Zero(4, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  ekf_.x_ = VectorXd(4);
  ekf_.Init(ekf_.x_, P_, F_, H_laser_, R_laser_, Q_);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  long long ts;
  ts = measurement_pack.timestamp_;
  
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
    
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ << 1, 1, 1, 1;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      double px = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      double py = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
      double vx = measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]);
      double vy = measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);
      
      ekf_.x_ << px, py, vx, vy;
    }
    else{
      // TODO: Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = ts;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  float interval = (ts - previous_timestamp_) / 1000000.0;
  float interval2 = pow(interval, 2.0);
  float interval3 = pow(interval, 3.0) / 2.0;
  float interval4 = pow(interval, 4.0) / 4.0;
  float noise_ax = 9;
  float noise_ay = 9;
  
  ekf_.F_(0, 2) = interval;
  ekf_.F_(1, 3) = interval;
  
  ekf_.Q_(0, 0) = interval4 * noise_ax;
  ekf_.Q_(0, 2) = interval3 * noise_ax;
  ekf_.Q_(1, 1) = interval4 * noise_ay;
  ekf_.Q_(1, 3) = interval3 * noise_ay;
  ekf_.Q_(2, 2) = interval2 * noise_ax;
  ekf_.Q_(2, 0) = interval3 * noise_ax;
  ekf_.Q_(3, 1) = interval3 * noise_ay;
  ekf_.Q_(3, 3) = interval2 * noise_ay;
 
  
  ekf_.Predict();
  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  
  Eigen::VectorXd z;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    z = VectorXd(3);
    z << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2];
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(z);
  } else {
    // TODO: Laser updates
    z = VectorXd(2);
    z << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1];
    
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
  
  previous_timestamp_ = ts;
}
