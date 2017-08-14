#include <cmath>
#include "vehicle.h"
#include "tools.h"


VehicleModel::VehicleModel() {
  is_initialized=false;

  //
  // Initialize all state vectors
  //
  e_ = VectorXd::Zero(2);
  e_previous = VectorXd::Zero(2);
  u_ = VectorXd::Zero(2);
  u_previous = VectorXd::Zero(2);
  x_ = VectorXd::Zero(6);
  x_previous = VectorXd::Zero(6);


}

VehicleModel::~VehicleModel() {}


VectorXd VehicleModel::PredictError(const double delta_t) {
  const double  epsi_1 = e_(1) + (x_(2)/Lf) * u_(0) * delta_t;
  const double  cte_1 = e_(0) + x_(2) * sin(e_(1)) * delta_t;
  VectorXd err = VectorXd (2);
  err << cte_1, epsi_1;
  return err;

}


VectorXd VehicleModel::PredictState(const double dt) {
  const double v = x_(2);
  const double yaw = x_(3);
  const double a = x_(5);
  const double yawd = x_(6);
  VectorXd delta_x = VectorXd::Zero(n_x_);

  // delta represents the change
  delta_x << v * cos(yaw) * dt,
             v * sin(yaw) * dt,
             a * dt,
             yawd * dt,
             0,
             0;
  // which is be then added to state
  VectorXd new_x = x_ + delta_x;
  return new_x;
}


void VehicleModel::UpdateError(VectorXd poly_coeffs){
  e_(0) = polyeval(poly_coeffs, 0);
  e_(1) = -atan(poly_coeffs[1]);
}


void VehicleModel::UpdateState(const MeasurementPackage* meas_pack) {
  VectorXd new_e = VectorXd(n_e_);
  VectorXd new_x = VectorXd(n_x_);
  VectorXd new_u = VectorXd(n_u_);

  new_e << 0.0, 0.0; // Errors will be calculated later
  // Extract information from measurement pack
  new_x << meas_pack->raw_measurements_[0],  // px
           meas_pack->raw_measurements_[1],  // py
           meas_pack->raw_measurements_[2],  // abs. velocity
           meas_pack->raw_measurements_[3],  // yaw-angle
           0.0,  // acceleration (calculate later)
           0.0;  // yaw_rate (calculate later)
  // Estimate yaw-rate
  new_x(5) =(new_x(2)/Lf) * new_u(0);

  new_u << meas_pack->raw_measurements_[4],  // steering angle
           meas_pack->raw_measurements_[5];  // throttle

  if (!is_initialized) {
    // Initialize this and previous with current values
    e_ = new_e;
    e_previous = new_e;
    x_ = new_x;
    x_previous = new_x;
    u_ = new_u;
    u_previous = new_u;
  }
  else {
    // Save current and previous state information
    e_previous = e_;
    e_ = new_e;
    x_previous = x_;
    x_ = new_x;
    u_previous = u_;
    u_ = new_u;
    // Same for timesteps
    timestamp_previous_ = timestamp_;
    timestamp_ = meas_pack->timestamp_;

    // Calculate real current acceleration
    const double dt = (timestamp_ - timestamp_previous_).count();
    x_(4) = (x_(2) - x_previous(2)) / dt;
    }
}

void VehicleModel::UpdateWaypoints(const vector<VectorXd> waypoints) {
  // Copy vector
  vector<VectorXd> nw;
  nw.push_back(VectorXd(waypoints[0]));
  nw.push_back(VectorXd(waypoints[1]));
  waypoints_global = nw;
  waypoints_local = transformWaypoints(waypoints_global, x_(0), x_(1), x_(3));
  waypoint_poly_coeffs = polyfit(waypoints_local[0], waypoints_local[1], 3);
}
