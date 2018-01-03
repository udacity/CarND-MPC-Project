#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "helper_functions.hpp"

using CppAD::AD;

// TODO: Set the timestep length and duration
const size_t N = 10;
double dt = 0.1;
const size_t xstart=0*N;
const size_t ystart=1*N;
const size_t psistart=2*N;
const size_t vstart=3*N;
const size_t ctestart=4*N;
const size_t epsistart=5*N;
const size_t deltastart=6*N;
const size_t astart=6*N + 1*(N-1);


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
const double v_set=50;
const double set_speed_factor=0;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }


  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    
    // set the cost in fg[0]
    fg[0]=0;

    for (int i=0;i<N;i++) {
      AD<double> setspeed=v_set-CppAD::abs( vars[deltastart+i]*set_speed_factor);

      fg[0] += CppAD::pow(vars[ctestart+i]/1,2);
      fg[0] += CppAD::pow(vars[epsistart+i]/2,2);
      fg[0] += CppAD::pow((vars[vstart+i]-setspeed)/5,2);

    }
    
    for (int i = 0; i < N - 1; i++) {
      fg[0] += CppAD::pow(vars[deltastart + i]/100., 2);
      fg[0] += CppAD::pow(vars[astart + i]/1000., 2);
      // try adding penalty for speed + steer
    }
    for (int i = 0; i < N - 2; i++) {
      fg[0] += CppAD::pow((vars[deltastart + i + 1] - vars[deltastart + i])/.1, 2);
      fg[0] += CppAD::pow((vars[astart + i + 1] - vars[astart + i])/.01, 2);
    }
    
    
    fg[1 + xstart] = vars[xstart];
    fg[1 + ystart] = vars[ystart];
    fg[1 + psistart] = vars[psistart];
    fg[1 + vstart] = vars[vstart];
    fg[1 + ctestart] = vars[ctestart];
    fg[1 + epsistart] = vars[epsistart];
    
    
    
    for (int t = 1; t < N; t++) {
      AD<double> x1 = vars[xstart + t];
      AD<double> y1 = vars[ystart + t];
      AD<double> v1= vars[vstart +t];
      AD<double> psi1 = vars[psistart +t];
      AD<double> cte1= vars[ctestart +t];
      AD<double> epsi1=vars[epsistart+t];

      AD<double> x0 = vars[xstart + t -1];
      AD<double> y0 = vars[ystart + t-1];
      AD<double> v0= vars[vstart +t-1];
      AD<double> psi0 = vars[psistart +t-1];
      AD<double> cte0= vars[ctestart +t-1];
      AD<double> epsi0=vars[epsistart+t-1];
      
      AD<double> delta0;
      AD<double> a0;
      if (t == 1) {
        delta0= vars[deltastart +t-1];
        a0=vars[astart+t-1];
      } else { // for latency
        a0 = vars[astart + t - 2];
        delta0 = vars[deltastart + t - 2];
      }
      
      
      AD<double> fx0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));// atan of the derivative
      
      // TODO: Setup the rest of the model constraints
      fg[1 + xstart + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + ystart + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psistart + t]= psi1 - (psi0  - v0/Lf*delta0*dt);
      fg[1 + vstart + t] = v1 - (v0 + a0 * dt);
      // include drag term
      //fg[1 + vstart + t] = v1 - (v0 + (a0-0.4*v0*v0/50./50.) * dt);

      fg[1 + ctestart +t] = cte1-(fx0 - y0 + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsistart +t] = epsi1-(psi0-psides0 - v0 / Lf * delta0 * dt);
    }
    
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    

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
  // 6 * N + 2 * (N-1)
  // order is N*x, N*y, N*psi, N*v, N*cte, N*epsi, (N-1)*delta, (N-1)*a
  size_t n_vars = 6*N +2*(N-1);
  // TODO: Set the number of constraints
  size_t n_constraints = 6*N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for ( i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  vars[xstart]=state[0];//x
  vars[ystart]=state[1];//y
  vars[psistart]=state[2];//psi
  vars[vstart]=state[3];//v
  vars[ctestart]=state[4];//cte
  vars[epsistart]=state[5];//epsi

  // TODO: Set lower and upper limits for variables.

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for( i=xstart;i<xstart+N;++i) { //x
    vars_lowerbound[i]=-1E6;
    vars_upperbound[i]=1E6;
  }
  for( i=ystart;i<ystart+N;++i) { //y
    vars_lowerbound[i]=-1E6;
    vars_upperbound[i]=1E6;
  }
  for( i=psistart;i<psistart+N;++i) { //psi
    vars_lowerbound[i]=-1E6;
    vars_upperbound[i]=1E6;
  }
  for( i=vstart;i<vstart+N;++i) { //v
    vars_lowerbound[i]=-1E6;
    vars_upperbound[i]=1E6;
  }

  for( i=ctestart;i<ctestart+N;++i) { //cte
    vars_lowerbound[i]=-1E6;
    vars_upperbound[i]=1E6;
  }

  for( i=epsistart;i<epsistart+N;++i) { //epsi
    vars_lowerbound[i]=-1E6;
    vars_upperbound[i]=1E6;
  }

  for( i=astart;i<astart+N-1;++i) { //a
    vars_lowerbound[i]=-1;
    vars_upperbound[i]=1;
  }
  for( i=deltastart;i<deltastart+N-1;++i) { //delta
    vars_lowerbound[i]=-deg2rad(25);
    vars_upperbound[i]=deg2rad(25);
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for ( i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  constraints_lowerbound[xstart] = state[0];
  constraints_lowerbound[ystart] = state[1];
  constraints_lowerbound[psistart] = state[2];
  constraints_lowerbound[vstart] = state[3];
  constraints_lowerbound[ctestart] = state[4];
  constraints_lowerbound[epsistart] = state[5];
  
  constraints_upperbound[xstart] = state[0];
  constraints_upperbound[ystart] = state[1];
  constraints_upperbound[psistart] = state[2];
  constraints_upperbound[vstart] = state[3];
  constraints_upperbound[ctestart] = state[4];
  constraints_upperbound[epsistart] = state[5];

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
  std::cout << "Control: "<< solution.x[deltastart] <<", "<<solution.x[astart]<<std::endl;
  std::cout << "Setspeed: "<<v_set-std::abs(solution.x[deltastart]*set_speed_factor)<<std::endl;
  //std::cout << "Solution: "<< solution.x <<std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  //std::cout <<" Solution size: " << solution.x.size()<<std::endl;
  //std::cout << solution.x<<std::endl;
  vector<double> results(2+2*N);
  results[0]=solution.x[deltastart];
  results[1]=solution.x[astart];
  for (i=0;i<N;i++) {
    results[2+2*i]=solution.x[xstart+i];
    results[2+2*i+1]=solution.x[ystart+i];
  }
  return results;
}
