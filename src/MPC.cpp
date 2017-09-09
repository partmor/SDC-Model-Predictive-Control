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
      for (size_t t = 0; t < N; t++) {

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
        fg[1 + x_start + t + 1] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_start + t + 1] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_start + t + 1] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
        fg[1 + v_start + t + 1] = v1 - (v0 + a0 * dt);
        fg[1 + cte_start + t + 1] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
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
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 0;
  // TODO: Set the number of constraints
  size_t n_constraints = 0;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

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
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {};
}
