#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;

size_t N = 10;
double dt = 0.18;

const double Lf = 2.67;
double ref_v = 70;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

AD<double> polyeval(Eigen::VectorXd coeffs, AD<double> x) {
  AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

class FG_eval {
public:
    // Coefficients of the fitted polynomial.
    Eigen::VectorXd coeffs;

    FG_eval(Eigen::VectorXd coeffs, double target_x, double target_y) {
      this->coeffs = coeffs;
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    // `fg` is a vector containing the cost and constraints.
    // `vars` is a vector containing the variable values (state & actuators).
    void operator()(ADvector& fg, const ADvector& vars) {
      // The cost is stored is the first element of `fg`.
      fg[0] = 0;

      fg[1 + x_start] = vars[x_start];
      fg[1 + y_start] = vars[y_start];
      fg[1 + psi_start] = vars[psi_start];
      fg[1 + v_start] = vars[v_start];
      fg[1 + cte_start] = vars[cte_start];
      fg[1 + epsi_start] = vars[epsi_start];

      // The rest of the constraints
      for (int t = 1; t < N; t++) {
        AD<double> x1 = vars[x_start + t];
        AD<double> y1 = vars[y_start + t];
        AD<double> psi1 = vars[psi_start + t];
        AD<double> v1 = vars[v_start + t];
        AD<double> cte1 = vars[cte_start + t];
        AD<double> epsi1 = vars[epsi_start + t];

        AD<double> x0 = vars[x_start + t - 1];
        AD<double> y0 = vars[y_start + t - 1];
        AD<double> psi0 = vars[psi_start + t - 1];
        AD<double> v0 = vars[v_start + t - 1];
        AD<double> delta0 = vars[delta_start + t - 1];
        AD<double> a0 = vars[a_start + t - 1];
        AD<double> cte0 = vars[cte_start + t - 1];
        AD<double> epsi0 = vars[epsi_start + t - 1];

        AD<double> y_fit = polyeval(coeffs, x0);
        AD<double> epsi = CppAD::atan(coeffs[1] + coeffs[2] * 2 * x0 + coeffs[3] * 3 * pow(x0, 2));

        // Define cost for the car
        fg[0] += CppAD::pow(cte0, 2) * 3000;
        fg[0] += CppAD::pow(epsi0, 2) * 750;
        fg[0] += CppAD::pow(ref_v - v0, 2) * 0.7;
        fg[0] += CppAD::pow(delta0, 2) * 50;
        fg[0] += CppAD::pow(a0, 2) * 30;


        if (t < N - 1) {
          AD<double> delta1 = vars[delta_start + t];
          AD<double> a1 = vars[a_start + t];
          fg[0] += CppAD::pow(delta1 - delta0, 2) * 50;
          fg[0] += CppAD::pow(a1 - a0, 2) * 30;
        }

        // Setup next state constraints
        fg[1 + x_start + t]    = x1    - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_start + t]    = y1    - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_start + t]  = psi1  - (psi0 + (v0 / Lf) * delta0 * dt);
        fg[1 + v_start + t]    = v1    - (v0 + a0 * dt);
        fg[1 + cte_start + t]  = cte1  - ((y_fit - y0) + (v0 * CppAD::sin(epsi0) * dt));
        fg[1 + epsi_start + t] = epsi1 - (epsi0 - epsi + (v0 / Lf) * delta0 * dt);
      }
    }
};

//
// MPC class definition
//

MPC::MPC() {
  ptsx = std::vector<double>(4);
  ptsy = std::vector<double>(4);
}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs, double target_x, double target_y) {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = x0[0];
  double y = x0[1];
  double psi = x0[2];
  double v = x0[3];
  double cte = x0[4];
  double epsi = x0[5];

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
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

  // Object that computes objective and constraints
  FG_eval fg_eval(coeffs, target_x, target_y);

  // options
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
          options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
          constraints_upperbound, fg_eval, solution);

  //
  // Check some of the solution values
  //

  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  for (int i = 0; i < 4; i++) {
    ptsx[i] = solution.x[x_start + i];
    ptsy[i] = solution.x[y_start + i];
  }

  return { solution.x[delta_start], solution.x[a_start]};
}