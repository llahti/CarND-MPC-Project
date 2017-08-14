#include <iostream>
#include "tools.h"
#include "vehicle.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
// for convenience
using json = nlohmann::json;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  VectorXd rmse(4);
  rmse.fill(0.0);

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
     || estimations.size() == 0){
     cout << "Invalid estimation or ground_truth data" << endl;
     return rmse;
  }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){

    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}


/**
 * @brief NormalizeAngle Normalizes angle to be between -PI and ṔI
 * @param angle
 * @return normalized angle
 */
double Tools::NormalizeAngle(double angle){
  while (angle >  M_PI) angle-=2.*M_PI;
  while (angle < -M_PI) angle+=2.*M_PI;
  return angle;
}


MeasurementPackage getStateFromJSON(nlohmann::json* obj) {
  std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
  timestamp = std::chrono::high_resolution_clock::now();
  // Extract data from JSON package
  VectorXd measurements(6);
  measurements.fill(0.0);
  measurements(0) = (*obj)[1]["x"];
  measurements(1) = (*obj)[1]["y"];
  measurements(2) = 0.44704 *(double)(*obj)[1]["speed"]; // convert mph to meters/second
  measurements(3) = (*obj)[1]["psi"];
  measurements(4) = -(double)(*obj)[1]["steering_angle"];  // Invert steering angle (delta)
  measurements(5) = (*obj)[1]["throttle"];
  // Finalize measurement pack and return it
  MeasurementPackage meas_pack = MeasurementPackage();
  meas_pack.raw_measurements_ = measurements;
  meas_pack.sensor_type_ = MeasurementPackage::SensorType::MPC_PRJ_SIM;
  meas_pack.timestamp_ = timestamp;
  return meas_pack;
}


vector<VectorXd> getWaypointsFromJSON(nlohmann::json* obj) {
  // Get waypoints from JSON object
  vector<double> ptsx = (*obj)[1]["ptsx"];
  vector<double> ptsy = (*obj)[1]["ptsy"];
  // Store in vector of vectors
  VectorXd ptsx_transform = Eigen::Map<VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());;
  VectorXd ptsy_transform = Eigen::Map<VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());;
  vector<VectorXd> points = {ptsx_transform, ptsy_transform};

  return points;
}


void printState(VectorXd* x, string tag) {
  std::cout << tag << ":"
            << "  x=" << (*x)(0)
            << ", y=" << (*x)(1)
            << ", v=" << (*x)(2)
            << ", y=" << (*x)(3)
            << ", d=" << (*x)(4)
            << ", t=" << (*x)(5)
            << ", yd=" << (*x)(6);
}

vector<VectorXd> transformWaypoints(const vector<VectorXd> waypoints,
                                    const double vehicle_x, const double vehicle_y,
                                    const double yaw) {
  // New vectors to store transformed points
  size_t size = waypoints[0].size();
  VectorXd wpx = VectorXd(size);
  VectorXd wpy = VectorXd(size);

  for (uint i=0; i < size; i++) {
      const double new_x = waypoints[0][i] - vehicle_x;
      const double new_y = waypoints[1][i] - vehicle_y;
      wpx(i) = (new_x*cos(-yaw) - new_y*sin(-yaw));
      wpy(i) = (new_x*sin(-yaw) + new_y*cos(-yaw));
  }

  vector<VectorXd> transformed = {wpx, wpy};
  return transformed;
}

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(VectorXd xvals, VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}
