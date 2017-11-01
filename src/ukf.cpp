#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
* Initializes Unscented Kalman filter
*/
UKF::UKF() {
  use_laser_ = true;   // if this is false, laser measurements will be ignored (except during init)
  use_radar_ = true;   // if this is false, radar measurements will be ignored (except during init)
  x_ = VectorXd(5);    // initial state vector
  P_ = MatrixXd(5, 5); // initial covariance matrix
  std_a_ = 2;          // Process noise standard deviation longitudinal acceleration in m/s^2
  std_yawdd_ = 0.3;    // Process noise standard deviation yaw acceleration in rad/s^2
  std_laspx_ = 0.15;   // Laser measurement noise standard deviation position1 in m
  std_laspy_ = 0.15;   // Laser measurement noise standard deviation position2 in m
  std_radr_ = 0.3;     // Radar measurement noise standard deviation radius in m
  std_radphi_ = 0.03;  // Radar measurement noise standard deviation angle in rad
  std_radrd_ = 0.3;    // Radar measurement noise standard deviation radius change in m/s

  /**
  TODO:
  Complete the initialization. See ukf.h for other member properties.
  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;                     // initially set to false, set to true in first call of ProcessMeasurement
  time_us_ = 0.0;                              // time when the state is true
  n_x_ = 5;                                    // state dimension
  n_aug_ = 7;                                  // augmented state dimension
  lambda_ = 3 - n_x_;                          // sigma point spreading parameter
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1); // predicted sigma points matrix
  weights_ = VectorXd(2 * n_aug_ + 1);         // create weights vector
  NIS_radar_ = 0.0;                            // current NIS for radar
  NIS_laser_ = 0.0;                            // current NIS for laser
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  
  if ((meas_package.sensor_type_ == MeasurementPackage:: RADAR && use_radar_) ||
(meas_package.sensor_type_ == MeasurementPackage:: LASER && use_laser_)){ // skip if sensor type is ignored

  if (!is_initialized_){
    // Initialize state x_ with first measurement
    x_ << 1, 1, 1, 1, 0.1;

    // initial covariance matrix
    P_ << 0.15, 0, 0, 0, 0,
          0,  0.15, 0, 0, 0,
          0,    0,  1, 0, 0,
          0,    0,  0, 1, 0,
          0,    0,  0, 0, 1;
    
    time_us_ = meas_package.timestamp_; // initial timestamp

    if (meas_package.sensor_type_ == MeasurementPackage:: LASER & use_laser_){
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
    } 
    else if (meas_package.sensor_type_ == MeasurementPackage:: RADAR && use_radar_){  // Convert radar to cartesian coordinates
      float ro = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
      float ro_do = meas_package.raw_measurements_(2);
      x_(0) = ro * cos(phi);
      x_(1) = ro * sin(phi);
    }
    is_initialized_ = true;
    return;
  }

 /*
  * PREDICTION
  */
 float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0; // change in time in seconds
 time_us_ = meas_package.timestamp_;

 Prediction(delta_t); // call prediction

 /*
  * UPDATE
  */
 if (meas_package.sensor_type_ == MeasurementPackage::LASER)
 {
   UpdateLidar(meas_package);
 }
 else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
 {
   UpdateRadar(meas_package);
 }
}
}

void UKF::Prediction(double delta_t) {
  /*
  * GENERATE SIGMA POINTS
  */
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1); // create sigma point matrix
  MatrixXd A = P_.llt().matrixL();              // calculate square root of P
  lambda_ = 3 - n_x_;                           // set lambda for non-augmented sigma points
  Xsig.col(0) = x_;                             // set first column of sigma point matrix
  for (int i = 0; i < n_x_; i++)
  { //set remaining sigma points
    Xsig.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
    Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
  }
 /*
  * GENERATE AUGMENTED SIGMA POINTS
  */
  VectorXd x_aug_ = VectorXd(n_aug_);                   // create augmented mean vector
  MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);           // create augmented state covariance
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1); // create sigma point matrix
  lambda_ = 3 - n_aug_;                                 // set lambda for augmented sigma points
  x_aug_.head(5) = x_;                                  // create augmented mean state
  x_aug_(5) = 0;                                        // set to zero
  x_aug_(6) = 0;                                        // set to zero

  P_aug_.fill(0.0);                       // create augmented covariance matrix
  P_aug_.topLeftCorner(5, 5) = P_;        // keep top 5,5 values
  P_aug_(5, 5) = std_a_ * std_a_;         // add noise value matrix
  P_aug_(6, 6) = std_yawdd_ * std_yawdd_; // add noise value to matrix

  MatrixXd L = P_aug_.llt().matrixL(); // create square root matrix

  Xsig_aug.col(0) = x_aug_; // create augmented sigma points
  for (int i = 0; i < n_aug_; i++)
  { // set augmented sigma points
    Xsig_aug.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
  }
 
  /*
  * PREDICT SIGMA POINTS
  */
  
  // Predict Sigma Points
  for (int i = 0; i< n_aug_; i++){ // extract values for better readability
    double P_x      = Xsig_aug(0, i);
    double P_y      = Xsig_aug(1, i);
    double v        = Xsig_aug(2, i);
    double yaw      = Xsig_aug(3, i);
    double yawd     = Xsig_aug(4, i);
    double nu_a     = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    double Px_p, Py_p;  // Predict state values

    // Avoid div by zero
    if (fabs(yawd) > 0.001)
    {
      Px_p = P_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      Py_p = P_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
  } else {
    Px_p = P_x + v * delta_t * cos(yaw);
    Py_p = P_y + v * delta_t * sin(yaw);
  }
  double v_p = v;
  double yaw_p = yaw + yawd * delta_t;
  double yawd_p = yawd;

  // Add noise
  Px_p = Px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
  Py_p = Py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
  v_p  = v_p + nu_a * delta_t;

  yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
  yawd_p = yawd_p + 0.5 * nu_yawdd * delta_t;

  // Write predicted sigma points into right column
  Xsig_pred_(0, i) = Px_p;
  Xsig_pred_(1, 1) = Py_p;
  Xsig_pred_(2, i) = v_p;
  Xsig_pred_(3, i) = yaw_p;
  Xsig_pred_(4, i) = yawd_p;
}
 
/*
  * CONVERT PREDICTED SIGMA POINTS TO MEAN/COVARIANCE
  */
  // set weights
  double weight_0 = lambda_/(lambda_ + n_aug_); // initialize weight0 variable
  weights_(0) = weight_0; //set first column to weight_0
  for (int i = 1; i < 2 * n_aug_+1; i++){ // set other columns
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }
  // set predict state mean
  x_.fill(0.0); // initialize with zero
  for (int i = 1; i < 2 * n_aug_+1; i++) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // set predict state covariance matrix
  P_.fill(0.0); // initialize with zero
  for (int i = 1; i < 2 * n_aug_+1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) > M_PI) x_diff(3) += 2. * M_PI;
    
    // calculate mean covariance
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}
/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */

  /**
  TODO:
  Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  You'll also need to calculate the lidar NIS.
  */

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  VectorXd z = meas_package.raw_measurements_;   // extract measurement using VectorXd
  int n_z = 2;                                   // set measurement dimensino for lidar (2D)
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1); // create matix for sigma points in measurement space
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) { // transform sigma points into measurement space
    double P_x = Xsig_pred_(0, i); // extract x values for readibility
    double P_y = Xsig_pred_(1, i); // extract y values for readibility
    
    Zsig(0, i) = P_x; // measurement model
    Zsig(1, i) = P_y; // measurement model
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  MatrixXd S = MatrixXd(n_z, n_z); // measurement covariance matrix S
  S.fill(0.0);                     // initialize S matrix with zeros
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;         // residual calculation
    S = S + weights_(i) * z_diff * z_diff.transpose(); // measurement covariance matrix
  }

  // calculate noise covariance matrix R
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_*std_laspx_, 0,
       0, std_laspy_*std_laspy_; 
  S = S + R; // add measurement noise covariance matrix

/*
 * UKF UPDATE FOR LIDAR
 */
  MatrixXd Tc = MatrixXd(n_x_, n_z); // create cross correlation matrix Tc
  Tc.fill(0.0); // initilaize with zeros
  for (int i = 0; i<2* n_aug_+1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred; // residual
    VectorXd x_diff = Xsig_pred_.col(i) - x_; // state difference
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  MatrixXd K = Tc * S.inverse(); // Kalman gain K
  VectorXd z_diff = z - z_pred; // residual
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff; // calculate NIS

x_ = x_ + K * z_diff; // update state mean
P_ = P_ + K * S * K.transpose(); // update covariance matrix
}


 /**
  TODO:
  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  You'll also need to calculate the radar NIS.
  */

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */

void UKF::UpdateRadar(MeasurementPackage meas_package) {
 
  VectorXd z = meas_package.raw_measurements_; // extract measurement as Vector using VectorXd
  int n_z = 3; // set measurement dimension
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_+1); // create matrix for sigma points in measurement space

  // transform sigma points into measurement space
  for (int i = 0; i< 2* n_aug_+1; i++){
    double P_x  = Xsig_pred_(0,i); // extract values for readibility
    double P_y  = Xsig_pred_(1,i);
    double v    = Xsig_pred_(2,i);
    double yaw  = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v; 
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(P_x * P_x + P_y*P_y); // rho
    Zsig(1,i) = atan2(P_y, P_x); // phi
    Zsig(2,i) = (P_x*v1 + P_y*v2) / sqrt(P_x*P_x + P_y*P_y); // rho dot
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i<2* n_aug_+1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i<2* n_aug_+1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred; // residual

    while (z_diff(1) > M_PI) z_diff(1)  -= 2.*M_PI; // angle normalization
    while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_*std_radr_,   0,    0,
                          0,    std_radphi_*std_radphi_,  0,
                          0,    0, std_radrd_*std_radrd_;
  S = S + R;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i< 2*n_aug_+1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred; // residual
    while (z_diff(1) > M_PI) z_diff(1)  -= 2.*M_PI; // angle normalization
    while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

    VectorXd x_diff = Xsig_pred_.col(i) - x_; // state difference
    while (x_diff(1) > M_PI) x_diff(1)  -= 2.*M_PI; // angle normalization
    while (x_diff(1) < -M_PI) x_diff(1) += 2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }


  MatrixXd K = Tc * S.inverse(); // Kalman gain K

  VectorXd z_diff = z - z_pred; // residual
  while (z_diff(1) > M_PI) z_diff(1)  -= 2.*M_PI; // angle normalization
  while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff; // calculate NIS

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}