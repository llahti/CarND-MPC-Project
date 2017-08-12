#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"
#include "json.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
using json = nlohmann::json;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
  double NormalizeAngle(double angle);
};

/**
 * @brief getStateFromJSON This function extracts data from JSON message coming from simulator
 * @param json JSON object
 * @return {MeasurementPackage} a MeasurementPackage where sensor_type_=MPC_PRJ_SIM
 * and raw_measurements_ include following measurements:
 * [0] px
 * [1] py
 * [2] velocity (meters / second)
 * [3] psi (Yaw or Orientation in radians)
 * [4] Steering angle in radians (converted into car local coordination)
 * [5] throttle value
 * [6] yaw-angle
 */
MeasurementPackage getStateFromJSON(nlohmann::json* obj);


/**
  Extracts waypoints from json object. Waypoints are in simulator global coordinate system.

 * @brief getWaypointsFromJSON extracts waypoints from json object.
 * @param obj JSON object
 * @return {vector} vector of 2 vectors
 * [0] {Eigen::VectorXd} x-positions
 * [1] {Eigen::VectorXd} y-positions
 */
vector<Eigen::VectorXd> getWaypointsFromJSON(nlohmann::json* obj);


/**
Simulator gives waypoint
  coordinates in a wordl-coordinates so those need to be
  transferred into car-coordinate system,

  Convert world coordinates to car-local coordinate system
  Rotate 90-degrees to right so that car front is pointing
  along x-axis --> This makes polynomial fit easier because
  then we are looking for horizontallish line
 * @brief transformWaypoints Transforms waypoints to vehicle local coordination system.
 * @param waypoints vector of x and y value vectors. Values are in type of Eigen::VectorXd.
 * @param vehicle_x Vehicles x-position
 * @param vehicle_y Vehicles y-position
 * @param yaw Vehicles yaw-angle
 * @return {vector} vector of 2 vectors
 * [0] {Eigen::VectorXd} x-positions
 * [1] {Eigen::VectorXd} y-positions
 */
vector<Eigen::VectorXd> transformWaypoints(const vector<Eigen::VectorXd> waypoints,
                                           const double vehicle_x,
                                           const double vehicle_y,
                                           const double yaw);

#endif /* TOOLS_H_ */
