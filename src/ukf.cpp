#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  
  use_laser_ = true; // if this is false, laser measurements will be ignored (except during init)
  use_radar_ = true; // if this is false, radar measurements will be ignored (except during init)
  is_initialized_ = false;


  x_ = VectorXd(5); // initial state vector
  P_ = MatrixXd(5, 5); // initial covariance matrix
  P_ << 1,0,0,0,0,
        0,1,0,0,0,
        0,0,1,0,0,
        0,0,0,1,0,
        0,0,0,0,1;
  
  // initial predicted sigma points in state space
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_aug_ = MatrixXd(n_aug_, 2*n_aug_+1);
  x_aug_ = MatrixXd(n_aug_);
  P_aug_ = MatrixXd(n_aug_, n_aug_);
  x_pred_ = VectorXd(n_x_);
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  mean_a_ = 0.0;
  mean_yawdd_ = 0.0;
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
   * End DO NOT MODIFY section for measurement noise values 
   */

  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  
   // set state dimension
  n_x_ = 5;

  // set augmented dimension
  n_aug_ = 7;

  // define spreading parameter
  lambda_ = 3 - n_aug_;

  // set vector for weights
  weights_ = VectorXd(2 * n_aug_+1);
  double weight_0 = lambda_/(lambda_+n_aug_);
  double weight = 0.5/(lambda_+n_aug_);
  weights_(0) = weight_0;

  for(int i=1; i < 2*n_aug_+1; i++) {
    weights_(i) = weight;
  }
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  
  // define the delta_t here 
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  //this includes two - three functions
  // generateSigmaPoints()
  // augmentSigmaPoints
  //predict mean and covariance 
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}

void UKF::GenerateSigmaPoints() {

  // This function requires x_ and P_ as inputs. 
  MatrixXd A = P_.llt().matrixL();
  MatrixXd A_final = sqrt(lambda_ + n_x_) * A;
  MatrixXd A_mid = A_final;
  MatrixXd A_last = A_final;

  for(int i=0; i < A_final.cols(); i++) {
    A_mid.col(i) = x_ + A_final.col(i);
    A_last.col(i) = x_ - A_final.col(i);
  }

  // This function returns a Xsig_ matrix. 
  MatrixXd Xsig_ = MatrixXd(n_x_, 2*n_x_+1);
  Xsig_ << x_, A_mid, A_last;
}

void UKF::AugmentedSigmaPoints() {
  x_aug_.head(5) = x_;
  x_aug_(5) = mean_a_;
  x_aug_(6) = mean_yawdd_;

  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5,5) = P_;
  P_aug_(5,5) = std_a_*std_a_;
  P_aug_(6,6) = std_yawdd_*std_yawdd_;

  MatrixXd L = P_aug_.llt().matrixL();

  // create augmented sigma points
  Xsig_aug_.col(0)  = x_aug_;
  for (int i = 0; i< n_aug_; ++i) {
    Xsig_aug_.col(i+1)       = x_aug_ + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_+n_aug_) * L.col(i);
  }
}

void UKF::SigmaPointPrediction(double delta_t) {
  
  for(int i=0; i < 2*n_aug_+1; i++) {
    float p_x               = Xsig_aug_(0,i);
    float p_y               = Xsig_aug_(1,i);    
    float v                 = Xsig_aug_(2,i);
    float yaw               = Xsig_aug_(3,i);
    float yawrate           = Xsig_aug_(4,i);
    float noise_a           = Xsig_aug_(5,i);
    float noise_yawrate     = Xsig_aug_(6,i);

    if(fabs(yawrate > 0.001)) {
      double radius = v/yawrate;

      p_x += radius * (sin(yaw + yawrate * delta_t) - sin(yaw)) + 
                  (0.5 * pow(delta_t, 2) * cos(yaw) * mean_a_);
      p_y += radius * (cos(yaw) - cos(yaw + yawrate * delta_t)) + 
                  (0.5 * pow(delta_t, 2) * sin(yaw) * mean_a_);
      v   += delta_t * mean_a_;
      yaw += (yawrate * delta_t) + (0.5 * pow(delta_t,2) * mean_yawdd_);
      yawrate += delta_t * mean_yawdd_;

    }

    else {
      p_x += (v*cos(yaw)*delta_t) + (0.5 * pow(delta_t, 2) * cos(yaw) * mean_a_);
      p_y += (v*sin(yaw)*delta_t) + (0.5 * pow(delta_t, 2) * sin(yaw) * mean_a_);
      v += delta_t * mean_a_;
      yaw += (yawrate * delta_t) + (0.5 * pow(delta_t,2) * mean_yawdd_);
      yawrate += delta_t * mean_yawdd_;
    }

    Xsig_pred_(0,i) = p_x;
    Xsig_pred_(1,i) = p_y;
    Xsig_pred_(2,i) = v;
    Xsig_pred_(3,i) = yaw;
    Xsig_pred_(4,i) = yawrate;
  }


}

void UKF::PredictMeanAndCovariance() {

  // MatrixXd 
  // predicted mean and the covariance 
  // x_pred_.fill(0);
  // for(int i=0; i < 2*n_aug_+1; i++) {
  //   x_pred_ += weights_[i] * 
  // }
}


