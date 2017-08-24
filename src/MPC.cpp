#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration.
// Timestep length is 100ms and number of timesteps is 12
// This makes our prediction horizon 1.2s long
size_t N = 8;
const double dt = 0.120;

// Look-ahead time is for predicting first point later in future
// and that way making predictions to take more into account vehicle dynamics.
// I.E. you need to turn wheels just before the turn in order not to be late
const double lh = 0.00; // Look-ahead time
double dyndt;  // dynamic dt to account look-ahead


// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 40 mph.
double ref_cte = 0;
double ref_epsi = 0;

// 40m/s should work quite well
// Don't use overly high speed because it will affect cost calculations
// may render costs from other courses useless and cause unstable path.
// Anyhow.. car's max. speed in simulator is around 115mph which is  (x m/s)?
double ref_v = 45;  // unit is m/s (meters per second)

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t psid_start = psi_start + N;
size_t v_start = psid_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  // Constructor which takes polynomial coefficients
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  /**
   * @brief operator () is needed for solver to work correctly
   * @param fg `fg` is a vector containing the cost and constraints.
   * @param vars `vars` is a vector containing the variable values (state & actuators).
   */
  void operator()(ADvector& fg, const ADvector& vars) {
    // `fg` a vector of the cost constraints, `vars` is a vector of
    // variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // The cost is stored in the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    // Use high weights for cte and epsi to emphasize that those
    // are important to keep close to 0.
    // TODO: Describe more in details
    for (uint t = 0; t < N-1; t++) {
      // Penalize cte and give higher value to future cte
      fg[0] += 50*CppAD::pow(vars[cte_start + t] - ref_cte, 2);  // Try 2000 as multiplier
      fg[0] += 20*(t+1)*CppAD::pow(vars[cte_start + t] - ref_cte, 2);  // Try 2000 as multiplier
      // Penalize epsi and give higher value to instantanous values
      fg[0] += 50*     CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);  // Try 2000 as multiplier
      fg[0] += 50*1/(t+1)*CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);  // Try 2000 as multiplier
      // deviation from reference speed need to be penalized, because otherwise car wouldn't move
      // Keep penalty for not meeting reference speed small because otherwise it'll
      // mask cost of more important parameters such like cte or epsi.
      fg[0] += 2*CppAD::pow(vars[v_start + t] - ref_v, 2);
      // Penalize high speed with high steering angle
      fg[0] += 0.5*CppAD::pow(vars[v_start + t] * vars[delta_start + t], 2);
    }


    // Minimize the use of actuators.
    // TODO: Describe more in details
    for (uint t = 0; t < N - 1; t++) {
      fg[0] += 5*CppAD::pow(vars[delta_start + t], 2);  // try 5 as a multiplier
      fg[0] += 5*CppAD::pow(vars[a_start + t], 2);  // try 5 as a multiplier
    }

    // Minimize the value gap between sequential actuations.
    // TODO: Describe more in details
    for (uint t = 0; t < N - 2; t++) {
      fg[0] += 200*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);  // Try 200
      fg[0] += 10*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);  // try 10
    }

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + psid_start] = vars[psid_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];


    // The rest of the constraints
    for (uint t = 0; t < N-1; t++) {
      // The state at time t+1 .
      AD<double> x1 =    vars[x_start    + t +1];
      AD<double> y1 =    vars[y_start    + t +1];
      AD<double> psi1 =  vars[psi_start  + t +1];
      AD<double> psid1 =  vars[psid_start  + t +1];
      AD<double> v1 =    vars[v_start    + t +1];
      AD<double> cte1 =  vars[cte_start  + t +1];
      AD<double> epsi1 = vars[epsi_start + t +1];

      // The state at time t.
      AD<double> x0 =     vars[x_start    + t];
      AD<double> y0 =     vars[y_start    + t];
      AD<double> psi0 =   vars[psi_start  + t];
      AD<double> psid0 =  vars[psid_start  + t];
      AD<double> v0 =     vars[v_start    + t];
      AD<double> cte0 =   vars[cte_start  + t];
      AD<double> epsi0 =  vars[epsi_start + t];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t];
      AD<double> a0 =     vars[a_start     + t];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*CppAD::pow(x0, 2));  // Desired psi


      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      // More accurate model for high yaw-rate

      // first iteration is with look-ahead
      if (t == 0) {dyndt = lh + dt;}
      else {dyndt = dt; }

      AD<double> f_steer = MPC::ds / (MPC::ds/MPC::ds_min + CppAD::pow(v0, 2));
      fg[2 + psid_start   + t] = psid1  - ((1 - f_steer) * psid0 + f_steer * (v0 / MPC::Lf) * CppAD::tan(delta0));

      if (psid0 > 0.001) {
        // CTRV model
        fg[2 + x_start   + t] = x1   - (x0 + v0/psid0 * ( CppAD::sin(psi0 + psid0 * dyndt) - CppAD::sin(psi0)));
        fg[2 + y_start   + t] = y1   - (y0 + v0/psid0 * (-CppAD::cos(psi0 + psid0 * dyndt) + CppAD::cos(psi0)));
      }
      else {
        // Straight line model
        fg[2 + x_start   + t] = x1   - (x0 + v0 * CppAD::cos(psi0) * dyndt);
        fg[2 + y_start   + t] = y1   - (y0 + v0 * CppAD::sin(psi0) * dyndt);
      }
      fg[2 + psi_start + t] = psi1 - (psi0 + psid0 * dyndt);
      fg[2 + v_start   + t] = v1   - (v0 + a0 * dyndt);
      fg[2 + cte_start + t] =
          cte1 - ((f0 - y0) - (v0 * CppAD::sin(epsi0) * dyndt));
      // TODO: Check the correct equation for epsi1
      fg[2 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) - psid0 * dyndt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {

  InitOptionsString();
}
MPC::~MPC() {}


void MPC::InitOptionsString()
{
  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  options="";
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";
}

// TODO: Describe in details and document what this is doing!
vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  //size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Extract state from vector
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double psid = state[3];
  double v = state[4];
  double cte = state[5];
  double epsi = state[6];

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:

  // Number of state and actuator variables
  const unsigned int n_state_vars = MPC::n_x_;
  const unsigned int n_actuator_vars = MPC::n_u_;

  // number of independent variables
  // N timesteps == N - 1 actuations
  const size_t n_vars = N * n_state_vars + (N - 1) * n_actuator_vars;
  // Number of constraints
  const size_t n_constraints = N * n_state_vars;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (uint i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (uint i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }


  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  // TODO: Check what is function of multiplying by Lf

  for (uint i = delta_start; i < a_start; i++) {
    //vars_lowerbound[i] = -0.436332 * Lf;
    //vars_upperbound[i] = 0.436332 * Lf;
    vars_lowerbound[i] = -0.25 * Lf;
    vars_upperbound[i] = 0.25 * Lf;
  }


  // Acceleration/deceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (uint i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -0.8;
    vars_upperbound[i] = 1.0;
  }



  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (uint i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  // Define starting values for solver
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[psid_start] = psid;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[psid_start] = psid;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  vector<double> result;
  // Steering angle and acceleration
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  // Future xy-values for visualizations
  for (uint i = 0; i < N-1; i++) {
      result.push_back(solution.x[x_start+i+1]);
      result.push_back(solution.x[y_start+i+1]);
    }
  return result;
}
