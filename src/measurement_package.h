#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include <chrono>
#include "Eigen/Dense"

class MeasurementPackage {
public:
  std::chrono::time_point<std::chrono::high_resolution_clock>  timestamp_;

  enum SensorType{
    UNKNOWN,  // Can be used to indicate e.g. uninitialized measurement package
    LASER,
    RADAR,
    MPC_PRJ_SIM   ///* raw_measurements: [px py vel_abs yaw_angle steer_angle acceleration yaw_rate]
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;

};

struct LaserMeasurement {
  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_  = 0.15;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_ = 0.15;
};

struct RadarMeasurement {
  ///* Radar measurement noise standard deviation radius in m
  double std_radr_ = 0.3;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_ = 0.03;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ = 0.3;
};

/**
 * @brief The SimulatorMeasurement struct standard deviations for data returned by simulator
 */
const struct SimulatorMeasurement {
  ///* X-position standard deviation
  double std_px_ = 0.01;

  ///* Y-position standard deviation
  double std_py_ = 0.01;

  ///* Yaw-angle (psi) standard deviation
  double std_yaw_ = 0.01;

  ///* speed standard deviation
  double std_speed_ = 0.01;

  ///* acceleration standard deviation
  double std_accel_ = 0.01;

  ///* steering angle standard deviation
  double std_steerangle_ = 0.01;

  ///* yaw-rate standard deviation
  double std_yawdd_ = 0.01;
} simulator_consts;

#endif /* MEASUREMENT_PACKAGE_H_ */
