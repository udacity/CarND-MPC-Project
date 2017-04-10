#include "MPC.h"
#include <cppad/cppad.h>
#include <cppad/ipopt/solve.hpp>

namespace {
using CppAD::AD;

// TODO: Set the timestep length and duration
size_t T = 0;
double dt = 0;

// NOTE: DON'T CHANGE THIS IT WAS CAREFULLY CHOSEN!!!
const double Lf = 2.67;

class FG_eval {
 public:
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& x) {
    // TODO: implement MPC
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
  }
};
}

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

tuple<vector<double>, vector<double>, double> MPC::Solve(vector<double> x0) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of variables (includes both states and inputs)
  size_t nx = 0;
  // TODO: Set the number of constraints
  size_t ng = 0;

  // Initial value of the independent variables.
  // Should be 0 besides initial state.
  Dvector xi(nx);
  for (int i = 0; i < nx; i++) {
    xi[i] = 0;
  }

  // Lower and upper limits for states, inputs
  Dvector xl(nx), xu(nx);

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector gl(ng), gu(ng);
  for (int i = 0; i < ng; i++) {
    gl[i] = 0;
    gu[i] = 0;
  }

  //
  // NOTE: Most of this stuff you don't have to worry about
  //

  // object that computes objective and constraints
  FG_eval fg_eval;

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
  // Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(options, xi, xl, xu, gl, gu, fg_eval,
                                        solution);
  //
  // Check some of the solution values
  //
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // TODO: return the next state, input and current cost.
  // Note {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0,3.0}
  // creates a 3 element double vector.
  // Fill in the appropriate values
  auto x1 = {0.0, 0.0, 0.0};
  auto u1 = {0.0, 0.0};
  auto cost = solution.objvalue;
  return std::make_tuple(x1, u1, cost);
}