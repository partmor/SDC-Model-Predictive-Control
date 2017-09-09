#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;


size_t N = 20;
double dt = 0.05;

// Lf is the length from the front of the car to CoG, it is a physical characteristic of the vehicle.
// It was obtained by measuring the radius formed by the vehicle in the simulator running around
// in a circle with a constant steering angle and velocity on a flat terrain:
// Lf was tuned until the the radius formed by the control model matched the actual radius from the
// simulator
const double Lf = 2.67;

// reference velocity used in penalty term to avoid stopping
double ref_v = 40;

// the Ipopt solver takes all the state and actuator variables (for all instants) in a single vector:
// [x0, ... , xN-1, y0, ... , yN-1, psi0, ..., psiN-1, v0, ... aN-2]
// set the indices where each variable starts
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N - 1;
size_t a_start = delta_start + N - 1;
// NOTE: actuator inputs (delta, a) have 1 component less than the state variables:
// actuator input "i" takes us to state "i+1", so if the last state is N-1, actuators can only span up to N-2
// in other words in N timesteps only N-1 actuations would occur

class FG_eval {
  public:

    // fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    // `fg` is the vector of constraints
    // `vars` is the vector of variables (state & actuators)
    void operator()(ADvector& fg, const ADvector& vars) {

      //
      // Cost function (stored in fg[0])
      //

      fg[0] = 0;

      for (size_t t = 0; t < N; t++) {
        // penalize deviations w.r.t. reference state
        fg[0] += CppAD::pow(vars[cte_start + t], 2);
        fg[0] += CppAD::pow(vars[epsi_start + t], 2);

        // penalize car stopping
        fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
      }

      // penalize magnitude of actuator inputs
      for (size_t t = 0; t < N - 1; t++) {
        fg[0] += CppAD::pow(vars[delta_start + t], 2);
        fg[0] += CppAD::pow(vars[a_start + t], 2);
      }

      // penalize change rate of actuator input
      for (size_t t = 0; t < N - 2; t++) {
        fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
      }

      //
      // Constraints (stored in fg[1:end])
      //

      // initial constraints
      // add 1 to each starting index in fg (recall index 0 belongs to the cost function)
      fg[1 + x_start] = vars[x_start];
      fg[1 + y_start] = vars[y_start];
      fg[1 + psi_start] = vars[psi_start];
      fg[1 + v_start] = vars[v_start];
      fg[1 + cte_start] = vars[cte_start];
      fg[1 + epsi_start] = vars[epsi_start];

      // rest of the constraints
      for (size_t t = 0; t < N - 1; t++) {

        // extract state at time t+1
        AD<double> x1 = vars[x_start + t + 1];
        AD<double> y1 = vars[y_start + t + 1];
        AD<double> psi1 = vars[psi_start + t + 1];
        AD<double> v1 = vars[v_start + t + 1];
        AD<double> cte1 = vars[cte_start + t + 1];
        AD<double> epsi1 = vars[epsi_start + t + 1];

        // extract state at time t
        AD<double> x0 = vars[x_start + t];
        AD<double> y0 = vars[y_start + t];
        AD<double> psi0 = vars[psi_start + t];
        AD<double> v0 = vars[v_start + t];
        AD<double> cte0 = vars[cte_start + t];
        AD<double> epsi0 = vars[epsi_start + t];

        // extract actuation at time t
        AD<double> delta0 = vars[delta_start + t];
        AD<double> a0 = vars[a_start + t];

        // evaluate reference line at time t
        AD<double> f0 = coeffs[0] + coeffs[1] * x0;

        // evaluate desired orientation at time t, as the tangential angle to the reference line
        AD<double> psides0 = CppAD::atan(coeffs[1]);

        // recall the model equations:

        // x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
        // y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
        // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
        // v_[t+1] = v[t] + a[t] * dt
        // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
        // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

        // the model equations are re-written LHS - RHS = 0:
        // the constraint is LHS - RHS, with lower and upper bounds equal to zero
        fg[1 + x_start    + t + 1] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_start    + t + 1] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_start  + t + 1] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
        fg[1 + v_start    + t + 1] = v1 - (v0 + a0 * dt);
        fg[1 + cte_start  + t + 1] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
        fg[1 + epsi_start + t + 1] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
      }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {

  typedef CPPAD_TESTVECTOR(double) Dvector;

  // extract initial inputed state
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // size of state vector at each one of the N timesteps: 6
  // size of actuator vector at each one of N-1 actuations: 2
  // thus, number of model variables -independent variables- (includes both states and actuator inputs):
  size_t n_vars = N * 6 + (N - 1) * 2;

  // size of state vector at each one of the N timesteps: 6
  // thus, number of constraints:
  size_t n_constraints = N * 6;

  // initialize independent variables
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // lower and upper limits for variables
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // set all non-actuators upper and lower limits to the max negative/positive value
  for (size_t i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = std::numeric_limits<double>::lowest();
    vars_upperbound[i] = std::numeric_limits<double>::max();
  }

  // upper/lower limits of delta are set to -25/25 deg
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = - 25 * M_PI / 180;
    vars_upperbound[i] = 25 * M_PI / 180;
  }

  // acceleration/deceleration upper and lower limits
  for (size_t i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }


  // lower and upper limits for the constraints
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

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
  FG_eval fg_eval(coeffs);


  // options for IPOPT solver
  std::string options;
  // more print information:
  options += "Integer print_level  0\n";
  // setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // currently the solver has a maximum time limit of 0.5 seconds, this can be changed:
  options += "Numeric max_cpu_time          0.5\n";

  // solution container
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // check some of the solution values
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;


  // return the first actuator values
  // {...} is shorthand for creating a vector
  return {solution.x[x_start + 1],   solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[v_start + 1],
          solution.x[cte_start + 1], solution.x[epsi_start + 1],
          solution.x[delta_start],   solution.x[a_start]};
}
