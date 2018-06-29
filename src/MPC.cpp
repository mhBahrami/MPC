#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <cmath>
#include <utility>

using CppAD::AD;

// Set the timestep length and duration
// size_t: type returned by sizeof, widely used to represent sizes and counts
const size_t N = 15;
const double dt = 0.1;

// Set cost factors
/// Tune cost factors
const int cost_cte_factor = 3000;
const int cost_epsi_factor = 500; // made initial portion etc much less snaky
const int cost_v_factor = 1;
const int cost_current_delta_factor = 1;
const int cost_diff_delta_factor = 200;
const int cost_current_a_factor = 1;
const int cost_diff_a_factor = 1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Reference cross-track error and orientation error = 0
double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 40;

// Mark when each variable starts for convenience
// since state and actuator variables are stored in one vector
// in the format 'x...(N)x y...(N)y...'
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;


class FG_eval {
public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;

  explicit FG_eval(Eigen::VectorXd coeffs) { this->coeffs = std::move(coeffs); }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    /* Calculates cost of current state and predicts future states.

     */
    // fg is a vector of cost and constraints,
    // vars is a vector containing state and actuator var values and constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    //
    // Reference state cost
    //

    // Initialise cost to zero
    fg[0] = 0;

    // Cost increases with distance from reference state
    // Multiply squared devation by 'cost factors' to adjust contribution of each deviation to cost
    for (size_t i = 0; i < N; i++) {
      fg[0] += cost_cte_factor*pow(vars[cte_start + i] - ref_cte, 2);
      fg[0] += cost_epsi_factor*pow(vars[epsi_start + i] - ref_epsi, 2);
      fg[0] += cost_v_factor*pow(vars[v_start + i] - ref_v, 2);
    }

    // Cost increases with use of actuators
    for (size_t i = 0; i < N - 1; i++) {
      fg[0] += cost_current_delta_factor*pow(vars[delta_start + i], 2);
      fg[0] += cost_current_a_factor*pow(vars[a_start + i], 2);
    }

    // Cost increases with value gap between sequential actuators
    for (size_t i=0; i < N-2; i++) {
      fg[0] += cost_diff_delta_factor*pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += cost_diff_a_factor*pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }

    //
    // Constraints
    //

    // Set initial state (take from vars)
    // Add 1 to each of the starting indices since cost is at fg[0]
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // Set predicted states at other timesteps (from eqns)
    // N - 1 because we're only predicting (N-1) times
    for (size_t i = 0; i < N - 1; i++) {
      // The state at time t+1 .
      const AD<double> x1 = vars[x_start + i + 1];
      const AD<double> y1 = vars[y_start + i + 1];
      const AD<double> psi1 = vars[psi_start + i + 1];
      const AD<double> v1 = vars[v_start + i + 1];
      const AD<double> cte1 = vars[cte_start + i + 1];
      const AD<double> epsi1 = vars[epsi_start + i + 1];

      // The state at time t.
      const AD<double> x0 = vars[x_start + i];
      const AD<double> y0 = vars[y_start + i];
      const AD<double> psi0 = vars[psi_start + i];
      const AD<double> v0 = vars[v_start + i];
      const AD<double> cte0 = vars[cte_start + i];
      const AD<double> epsi0 = vars[epsi_start + i];

      // Only consider the actuation at time t.
      const AD<double> delta0 = vars[delta_start + i];
      const AD<double> a0 = vars[a_start + i];

      const AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0,2) + coeffs[3] * CppAD::pow(x0,3);
      const AD<double> psides0 = CppAD::atan(coeffs[1] + (2 * coeffs[2] * x0) + (3 * coeffs[3]* CppAD::pow(x0,2) ));

      // Fill in fg with differences between actual and predicted states
      // add 2 to indices because (+1) from cost and (+1) because logging error of prediction (next timestep)
      // Recall the equations for the model:
      // x_[t]    = x[t-1]    + v[t-1] * cos(psi[t-1]) * dt
      // y_[t]    = y[t-1]    + v[t-1] * sin(psi[t-1]) * dt
      // psi_[t]  = psi[t-1]  - v[t-1] / Lf * delta[t-1] * dt
      // v_[t]    = v[t-1]    + a[t-1] * dt
      // cte[t]   = f(x[t-1]) - y[t-1]      + v[t-1] * sin(epsi[t-1]) * dt
      // epsi[t]  = psi[t]    - psides[t-1] - v[t-1] * delta[t-1] / Lf * dt
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] =
          epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
    }

  }
};

//
// MPC class definition implementation.
//
MPC::MPC() = default;

MPC::~MPC() = default;

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  /* Minimises cost. */
  typedef CPPAD_TESTVECTOR(double) Dvector;

  /// Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  // State: [x,y,psi,v,cte,epsi]
  // Actuators: [delta,a]
  size_t n_vars = 6 * N + 2 * (N - 1);
  ///Set the number of constraints
  size_t n_constraints = 6 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  // Get init state
  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  // Set initial variable values to init state
  // but will be implicitly set by upper and lower bounds later so can remove this
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  ///Setting the lower and upper limits for variables
  for (size_t i = 0; i < delta_start; i++) {
    vars_upperbound[i] = 1.0e19;
    vars_lowerbound[i] = -1.0e19;
  }

  // Steering angle (deltas)
  for (size_t i = delta_start; i < a_start; i++)
  {
    vars_upperbound[i] = M_PI/8; // max values allowed in simulator
    vars_lowerbound[i] = -M_PI/8;
  }

  // Acceleration
  for (size_t i = a_start; i < n_vars; i++)
  {
    vars_upperbound[i] = 1.0;
    vars_lowerbound[i] = -1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  // Set init state lower and upper limits
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(std::move(coeffs));

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
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


  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;


  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  ///Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  vector<double> result;

  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  for (size_t i = 0; i < N-1; i++)
  {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }
  return result;
}