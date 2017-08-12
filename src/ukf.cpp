#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;



UKF::UKF() {
  // if this is false, simulator data will be ignored (except during init)
  use_simdata_ = true;

  // Initial state is "not initialized". UKF is initialized properly during the very first measurement update.
  is_initialized_ = false;

  //
  previous_timestamp_ = std::chrono::high_resolution_clock::now();

  //
  // Definitions for matrix sizes
  //
  // Length of state matrix [px py yaw_angle yaw_rate vel_abs acceleration steer_angle]
  n_x_ = 7;
  n_aug_ = 9;  // Length of augmented matrix (2 positions for acceleration noise and yawd noise)
  n_sp_xaug_ =  2 * n_aug_ + 1;  // Number of sigma point vectors for augmented state matrix
  n_sp_x_ = 2 * n_x_ + 1;  // Number of sigma point vectors for state matrix
  lambda_ = double(3 - n_x_); // define spreading parameter lambda
  lambda_aug_ = 3 - n_aug_; // define spreading parameter lambda for augmented sigma-points


  // initial state vector
  // [px py yaw_angle yaw_rate vel_abs acceleration steer_angle]
  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  //
  // This vector can be reused to generate diagonal matrices size of (n_x_ , n_x_)
  VectorXd nx_diag = VectorXd(n_x_);

  // initial covariance matrix, fill diagonal with 0.5
  nx_diag.fill(0.5);
  P_ = MatrixXd(n_x_, n_x_);
  P_= nx_diag.asDiagonal();

  // Initialize covariance matrix for augmented sigmapoints
  P_aug_ = MatrixXd(n_aug_, n_aug_);
  P_aug_.fill(0.0);

  // Initialize radar measurement covariance matrix
  //S_rad_ = MatrixXd(n_z_rad_, n_z_rad_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;


  // Define what measurements are used in update
  nx_diag.fill(1.0);
  H_simulator_ = MatrixXd(n_x_, n_x_);
  H_simulator_ = nx_diag.asDiagonal();
  /*
  H_simulator_ << 1, 0, 0, 0, 0, 0, 0,
                  0, 1, 0, 0, 0, 0, 0,
                  0, 0, 1, 0, 0, 0, 0,
                  0, 0, 0, 1, 0, 0, 0,
                  0, 0, 0, 0, 1, 0, 0,
                  0, 0, 0, 0, 0, 1, 0,
                  0, 0, 0, 0, 0, 0, 1;
*/
  // Initialize lidar measurement covariance matrix
  //R_lidar_ = MatrixXd(2, 2);
  //R_lidar_ << pow(std_laspx_, 2), 0,
  //            0, pow(std_laspy_, 2);


  //VectorXd r = VectorXd(n_x_);
  nx_diag << pow(simulator_consts.std_px_, 2),
             pow(simulator_consts.std_py_, 2),
             pow(simulator_consts.std_speed_, 2),
             pow(simulator_consts.std_yaw_, 2),
             pow(simulator_consts.std_steerangle_, 2),
             pow(simulator_consts.std_accel_, 2),
             pow(simulator_consts.std_yawdd_, 2);
  R_simulator_ = MatrixXd(n_x_, n_x_);
  R_simulator_ = nx_diag.asDiagonal();


  // Initialize sigmapoints matrix
  //Xsig_ = MatrixXd(n_x_, n_sp_x_);
  //Xsig_.fill(0.0);

  // Initialize predicted sigma points with zero
  Xsig_pred_ = MatrixXd(n_x_, n_sp_xaug_);
  Xsig_pred_.fill(0.0);

  // Initialize augmented sigma points with zero
  //Xsig_aug_ = MatrixXd(n_aug_, n_sp_xaug_);
  //Xsig_aug_.fill(0.0);

  // Initialize weights
  weights_ = VectorXd(n_sp_xaug_);
  weights_.fill(0.5 / float(n_aug_ + lambda_aug_));
  weights_(0) = lambda_aug_/(lambda_aug_+n_aug_);

  // Initialize NIS
  //NIS_laser_ = 0.0;
  NIS_simulator_ = 0.0;

  // Number of measurements from simulator
  n_z_simulator_ = 7;
}

UKF::~UKF() {}


/**
 * @brief UKF::AugmentedSigmaPoints
 */
Eigen::MatrixXd UKF::AugmentedSigmaPoints() {
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);
  //create augmented mean state
  x_aug.head(7) = x_;
  x_aug(7) = 0;
  x_aug(8) = 0;

  //create augmented state covariance
  P_aug_.fill(0.0);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sp_xaug_);
  Xsig_aug.fill(0.0);

  //create augmented covariance matrix by inserting P_ matrix to top-left corner
  P_aug_.topLeftCorner(7,7) = P_;
  // Insert process noise to bottom right corner
  P_aug_(7,7) = std_a_*std_a_;
  P_aug_(8,8) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug_.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;  // Mean state
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_aug_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_aug_+n_aug_) * L.col(i);
  }

  return Xsig_aug;
}

/**
 * @brief UKF::FirstUpdate handles the very first update cycle and initializes state vector.
 * @param measurement_pack
 */
void UKF::FirstUpdate(MeasurementPackage measurement_pack) {
  if (measurement_pack.sensor_type_ == MeasurementPackage::MPC_PRJ_SIM) {
    /**
    Convert data from simulator [px py yaw_angle vel_abs steer_angle acceleration]
    to state vector [px py yaw_angle yaw_rate vel_abs acceleration steer_angle]
    */
    Eigen::VectorXd m = measurement_pack.raw_measurements_;
    x_ << measurement_pack.raw_measurements_[0],  // px
          measurement_pack.raw_measurements_[1],  // py
          measurement_pack.raw_measurements_[2],  // abs. velocity
          measurement_pack.raw_measurements_[3],  // yaw-angle
          measurement_pack.raw_measurements_[4],  // steering angle
          0.0,                                    // acceleration, not properly returned from simulator
          measurement_pack.raw_measurements_[6];  // yaw_rate
   }
  previous_timestamp_ = measurement_pack.timestamp_;
  previous_simdata = measurement_pack;

  // Initialization done, save first measurement as a current state.
  is_initialized_ = true;
  return;
}


/**
 * This method generates sigma-points and calculations depends on instance variables P_, n_x_, n_sp_x_ and x_.
 * @brief UKF::GenerateSigmaPoints
 * @return Sigma points matrix
 */
Eigen::MatrixXd UKF::GenerateSigmaPoints() {
  // Initialize sigmapoints matrix
  MatrixXd Xsig = MatrixXd(n_x_, n_sp_x_);
  Xsig.fill(0.0);
  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();
  //Xsig.fill(0.0);
  //set first column of sigma point matrix
  Xsig.col(0) = x_;

  //set remaining sigma points
  for (int i = 0; i < n_x_; i++)
  {
    Xsig.col(i+1)     = x_ + sqrt(lambda_+n_x_) * A.col(i);
    Xsig.col(i+1+n_x_) = x_ - sqrt(lambda_+n_x_) * A.col(i);
  }
  return Xsig;
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
Eigen::VectorXd UKF::Prediction(double delta_t) {
  // 1. Generate sigma points
  MatrixXd Xsig = GenerateSigmaPoints();
  // 2. Predict sigma points
  MatrixXd Xsig_aug = AugmentedSigmaPoints();
  MatrixXd Xsig_pred = SigmaPointPrediction(delta_t, &Xsig_aug);
  Xsig_pred_ = Xsig_pred;  // save for later measurement prediction step
  // 3. Predict mean and covariance matrix
  VectorXd new_x = PredictMeanAndCovariance(&Xsig_pred);
  return new_x;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /******************
   *  Initialization
   ******************/
  //std::cout << "Meas: " << meas_package.raw_measurements_ << std::endl;
  if (!is_initialized_) {
    // Init first measurement
    FirstUpdate(meas_package);
    return;
  }

  /***********
   * Predict
   ***********/
  //compute the time in seconds elapsed between the current and previous measurements
  std::chrono::duration<double> time_diff = meas_package.timestamp_ - previous_timestamp_;
  double dt = time_diff.count();  // dt - expressed in seconds
  if (dt < 0.0001) {
    std::cout << "UKF: Measurement time difference unreasonable small!\n";
  }
  // Predict k+1 state
  x_ = Prediction(dt);

  /**********
   * Update
   **********/
  if (meas_package.sensor_type_ == MeasurementPackage::MPC_PRJ_SIM && use_simdata_ == true) {
    // Simulator Data Update
    // update timestamp only when measurement is processed
    previous_timestamp_ = meas_package.timestamp_;
    // Estimate acceleration and yaw_rate from measurements
    std::chrono::duration<double> time_diff = meas_package.timestamp_ - previous_simdata.timestamp_;
    double delta_t = time_diff.count();
    double delta_v = meas_package.raw_measurements_(2) - previous_simdata.raw_measurements_(2);
    double a = 0;
    if (delta_t < 0.00001) {a = 0;}
    else { a = delta_v / delta_t; } // Calculate acceleration from speed change
    // yawd = (speed/Lf) * steering_angle
    //double yawd = (meas_package.raw_measurements_(2)/vehicle.Lf) * meas_package.raw_measurements_(4);
    // Other way to calculate yaw is by using previous yaw-angle
    double yawd = (meas_package.raw_measurements_(3) - previous_simdata.raw_measurements_(3)) / delta_t;
    meas_package.raw_measurements_(5) = a;
    meas_package.raw_measurements_(6) = yawd;
    // Update state with simulation data
    UpdateSimulator(meas_package);
    // Save measpackage for next round
    previous_simdata = meas_package;
  }
  //return;
}



/**
 * @brief UKF::SigmaPointPrediction
 * @param delta_t time difference between this and previus state
 */
Eigen::MatrixXd UKF::SigmaPointPrediction(double delta_t, Eigen::MatrixXd* Xsig_aug) {
  MatrixXd Xsig_pred = MatrixXd(n_x_, n_sp_xaug_);
  Xsig_pred.fill(0.0);

  //predict sigma points
  for(int i=0; i < n_sp_xaug_; i++){
    // Variables to store (previous state) xk values for easier access to them
    double p_x = (*Xsig_aug)(0, i);       // px
    double p_y = (*Xsig_aug)(1, i);       // py
    double v = (*Xsig_aug)(2, i);         // speed
    double yaw = (*Xsig_aug)(3, i);       // yaw-angle
    double delta = (*Xsig_aug)(4, i);     // delta-angle (steering)
    double a = (*Xsig_aug)(5, i);         // acceleration
    double yawd = (*Xsig_aug)(6, i);      // yaw-rate
    double nu_a = (*Xsig_aug)(7, i);      // accelaration noise
    double nu_yawdd = (*Xsig_aug)(7, i);  // yaw-rate noise

    //predicted state values
    double px_p, py_p;

    //avoid division by zero when angle change is zero or very close to it
    if (fabs(yawd) > 0.0001){
      // predict px_k+1 py_k+1
      px_p = p_x + (v / yawd) * ( sin(yaw + (yawd * delta_t)) - sin(yaw));
      py_p = p_y + (v / yawd) * ( + cos(yaw) - cos(yaw + (yawd * delta_t)));
    }
    else {
      // predict px_k+1 py_k+1 when yaw-rate is close to zero
      px_p = p_x + (v * delta_t * cos(yaw));
      py_p = p_y + (v * delta_t * sin(yaw));
    }

    // Predict remaining elements
    double a_p = a;
    double v_p = v + a * delta_t;
    double yaw_p = yaw + yawd*delta_t;
    double delta_p = delta;
    double yawd_p = yawd;

    // Add noise
    px_p = px_p + 0.5 * pow(delta_t, 2) * cos(yaw) * nu_a;  // Add noise to px_k+1
    py_p = py_p + 0.5 * pow(delta_t, 2) * sin(yaw) * nu_a;
    v_p  += delta_t * nu_a;
    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma points into i-th column
    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = delta_p;
    Xsig_pred(5,i) = a_p;
    Xsig_pred(6,i) = yawd_p;
  }
  return Xsig_pred;
}

Eigen::VectorXd UKF::PredictMeanAndCovariance(Eigen::MatrixXd* Xsig_pred) {
  //predict state mean
  VectorXd new_x = (*Xsig_pred) * weights_;

  //predict state covariance matrix
  P_.fill(0.0);
  for (int i=0; i < n_sp_xaug_; i++){
      // Calculate difference
      VectorXd x_diff = (*Xsig_pred).col(i) - new_x;
      // Normalize angle yaw and delta angles
      x_diff(3) = tools.NormalizeAngle(x_diff(3));
      x_diff(4) = tools.NormalizeAngle(x_diff(4));

      P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
  return new_x;
}


/**
 * @brief UKF::PredictSimulatorMeasurement
 * @param z_pred_out
 */
void UKF::PredictSimulatorMeasurement(VectorXd *z_pred_out) {
  //create matrix for sigma points in measurement space and get px and py
  MatrixXd Zsig = MatrixXd(n_z_simulator_, n_sp_xaug_);
  Zsig = Xsig_pred_.block(0, 0, n_z_simulator_, n_sp_xaug_);

  //calculate predicted measurement
  VectorXd z_pred = VectorXd(n_z_simulator_);
  z_pred = Zsig * weights_;

  //write result
  *z_pred_out = z_pred;
}

/**
 * Updates the state and the state covariance matrix using simulator data
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateSimulator(MeasurementPackage meas_package) {
  // Predict simulator data by using sigma-points
  VectorXd z_pred;
  PredictSimulatorMeasurement(&z_pred);
  //VectorXd y = z - z_pred;
  VectorXd y = meas_package.raw_measurements_ - z_pred;
  MatrixXd Ht = H_simulator_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_simulator_ * PHt + R_simulator_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  //long x_size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(n_x_, n_x_);
  P_ = (I_ - K * H_simulator_) * P_;

  // Calculate NIS
  NIS_simulator_ = y.transpose() * Si * y;
}
