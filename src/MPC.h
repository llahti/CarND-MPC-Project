#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 private:
  std::string options;
  // Initialize solver options
  void InitOptionsString();

 public:
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

  // Length of state vector
  static constexpr size_t n_x_= 7;
  static constexpr size_t n_u_= 2;

  // Coefficient to define how much steering is affected by yaw-rate
  // and how much by steering angle
  static constexpr double ds = 800;
  // defines maximum portion from steering angle based prediction
  // This is needed when speed is close to zero.
  static constexpr double ds_min = 0.7;

  MPC();

  virtual ~MPC();


  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  /**
   * @brief Solve solves the model given an initial state and polynomial coefficients.
   * @param state vector of current state of the vehicle. This vector contains following elements.
   *         [0] X-position
   *         [1] Y-position
   *         [2] psi yaw
   *         [3] psid yaw-rate
   *         [4] velocity
   *         [5] cte (Cross Track Error)
   *         [6] epsi (Error in steering angle)
   * @param coeffs coefficients of polynomial of waypoints to be followed
   * @return
   */
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
