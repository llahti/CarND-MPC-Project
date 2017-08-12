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
  MPC();

  virtual ~MPC();


  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  /**
   * @brief Solve solves the model given an initial state and polynomial coefficients.
   * @param state vector of current state of the vehicle. This vector contains following elements.
   *         [0] X-position
   *         [1] Y-position
   *         [2] psi orientation
   *         [3] velocity
   *         [4] cte (Cross Track Error)
   *         [5] epsi (Error in steering angle)
   * @param coeffs coefficients of polynomial of waypoints to be followed
   * @return
   */
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
