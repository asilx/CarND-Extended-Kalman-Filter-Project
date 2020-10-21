#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd error(4);
  error << 0,0,0,0;
  
   if (estimations.size() != ground_truth.size() || estimations.size() == 0 || ground_truth.size() == 0) {
    cout << "Error: Invalid data" << endl;
    return error;
  }
  for (unsigned int i = 0; i < ground_truth.size(); i++) {

    VectorXd current_error = estimations[i] - ground_truth[i];
    current_error = current_error.array()*current_error.array();
    error += current_error;
  }
  
  error = error/ground_truth.size();
  error = error.array().sqrt();
  
  return error;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  float element1 = px*px+py*py;
  float element2 = sqrt(element1);
  float element3 = (element1*element2);
  
  // check division by zero
  if (fabs(element1) < 0.00001) {
    cout << "Exception at Jacobian: divided by zero!" << endl;
  }
  else {
  // compute the Jacobian matrix
    Hj << (px/element2), (py/element2), 0, 0,
          -(py/element1), (px/element1), 0, 0,
          py*(vx*py - vy*px)/element3, px*(px*vy - py*vx)/element3, px/element2, py/element2;
  }
  return Hj;
}
