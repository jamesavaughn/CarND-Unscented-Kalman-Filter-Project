#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd RMSE(4); //may need to change 4 to estimations.size()
  RMSE << 0,0,0,0;

  // check validity of inputs
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "######## Invalid Estimation or Ground Truth Data ########" << endl;
    return RMSE;
  }

  // accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i{
    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array() * residual.array();
    RMSE += residual;
  }

  // calculate mean
  RMSE = RMSE / estimations.size();

  // return result
  return RMSE;
}