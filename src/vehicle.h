#ifndef VEHICLE_H
#define VEHICLE_H

#include "Eigen-3.3/Eigen/Core"
#include "measurement_package.h"
#include <chrono>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class VehicleModel {
private:

public:
  //
  // Public variables
  //

  // This value assumes the model presented in the classroom is used.
  //
  // It was obtained by measuring the radius formed by running the vehicle in the
  // simulator around in a circle with a constant steering angle and velocity on a
  // flat terrain.
  //
  // Lf was tuned until the the radius formed by the simulating the model
  // presented in the classroom matched the previous radius.
  //
  // This is the length from front to CoG(Center of Gravity) that has a similar radius.
  static constexpr double Lf = 2.67;
  // This is pure guess of Lr value
  static constexpr double Lr = 2.67;

  // After first update turn this to true
  bool is_initialized=false;

  VectorXd waypoint_poly_coeffs;

  ///* Errors
  /// [0] cte (Cross Track Error)
  /// [1] epsi (Error on heading, yaw, psi)
  const int n_e_ = 2;
  VectorXd e_;
  VectorXd e_previous;

  ///* Controls
  /// [0] Steering Angle
  /// [1] throttle value
  const int n_u_ = 2;
  VectorXd u_;
  VectorXd u_previous;

  ///* state vector: in SI units and rad
  /// [0] X-position in meters
  /// [1] Y-position in meters
  /// [2] abs. velocity (speed in meters/second)
  /// [3] yaw_angle (heading in radians)
  /// [4] acceleration (meters/seconds^2)
  /// [5] yaw_rate (radians/second)
  const int n_x_ = 6;
  VectorXd x_;
  VectorXd x_previous;

  // Timestamps
  chrono::time_point<std::chrono::high_resolution_clock> timestamp_;
  chrono::time_point<std::chrono::high_resolution_clock> timestamp_previous_;


  // Store global and local waypoints here
  // Global = simulator coordinate system
  // Local = car's coordinate system
  vector<VectorXd> waypoints_global;
  vector<VectorXd> waypoints_local;

  //
  // Public Methods
  //

  VehicleModel();

  ~VehicleModel();

  VectorXd PredictError(const double delta_t);


  VectorXd PredictState(const double delta_t);



  /**
   * @brief UpdateError updates error vectors based on given polynomial coefficients
   * @param poly_coeffs 3rd order polynomial coefficients in a vector
   */
  void UpdateError(const VectorXd poly_coeffs);

  /**
   * @brief UpdateState Updates state information. Save old state as previous state.
   * @param meas_pack
   */
  void UpdateState(const MeasurementPackage* meas_pack);


  /**
   * @brief UpdateWaypoints Updates global and local waypoints and calculates 3rd order polynomial
   * @param waypoints vector of vectors
   * [0] x-points
   * [1] y-points
   */
  void UpdateWaypoints(const vector<VectorXd> waypoints);

};



#endif // VEHICLE_H
