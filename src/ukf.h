#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <chrono>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {

private:
  // previous timestamp
  std::chrono::time_point<std::chrono::high_resolution_clock> previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;

  // Previous simulation data pack is needed for acceleration calculations
  MeasurementPackage previous_simdata;

  struct VehicleModel {
    const double Lf = 2.67;
  } vehicle;

public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* Use simulator data
  bool use_simdata_;

  ///* state vector: in SI units and rad
  /// [0] px X-position in meters
  /// [1] py Y-position in meters
  /// [2] vel_abs speed in meters/second
  /// [3] yaw_angle heading in radians
  /// [4] delta_angle steering angle in radians
  /// [5] acceleration
  /// [6] yaw_rate
  VectorXd x_;

  ///* Previous state is used to calculate e.g. acceleration
  //VectorXd x_previous;

  ///* state covariance matrix
  MatrixXd P_;

  ///* state covariance matrix for augmented sigmapoints
  MatrixXd P_aug_;

  ///* Measurement covariance matrix for radar measurements
  //MatrixXd S_rad_;

  ///* sigma points matrix
  MatrixXd Xsig_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* augmented sigma points matrix
  MatrixXd Xsig_aug_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  /// Number of sigma-points for state matrix
  int n_sp_x_;

  /// Number of sigma-points for augmented state matrix
  int n_sp_xaug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* Augmented Sigma point spreading parameter
  double lambda_aug_;

  ///* Length of radar measurement vector
  //int n_z_rad_;

  ///* Length of lidar measurement vector
  //int n_z_lidar_;

  ///* Length of measurements from simulator
  int n_z_simulator_;

  ///* H matrix for Lidar update step
  //MatrixXd H_lidar_;
  MatrixXd H_simulator_;
  //MatrixXd R_lidar_;  // Measurement Covariance matrix for lidar
  MatrixXd R_simulator_;  // Measurement Covariance for simulator

  ///* NIS
  //double NIS_laser_;
  //double NIS_radar_;
  double NIS_simulator_;

  /**
   * Constructor
   */
  UKF();


  /**
   * Destructor
   */
  virtual ~UKF();

  void FirstUpdate(MeasurementPackage measurement_pack);


  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);


  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);


  /**
   * Updates the state and the state covariance matrix using a simulator measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateSimulator(MeasurementPackage meas_package);


  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  //void UpdateRadar(MeasurementPackage meas_package);


  /**
   * @brief GenerateSigmaPoints
   */
  void GenerateSigmaPoints();


  /**
   * @brief AugmentedSigmaPoints
   */
  void AugmentedSigmaPoints();

  void SigmaPointPrediction(double delta_t);

  void PredictMeanAndCovariance();

  //void PredictLidarMeasurement(VectorXd* z_pred_out);

  void PredictSimulatorMeasurement(VectorXd *z_pred_out);

  //void UpdateLidarState(MatrixXd* Zsig, VectorXd* z_pred, VectorXd* z);

};

#endif /* UKF_H */
