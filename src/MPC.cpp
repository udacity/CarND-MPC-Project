#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "helper_functions.hpp"
#include <chrono>
#include <thread>



using CppAD::AD;


const size_t xstart=0*N;   // first x position in vars, N in total
const size_t ystart=1*N;   // first y position in vars, N in total
const size_t psistart=2*N; // first psi in vars, N in total
const size_t vstart=3*N;   // first v in vars, N in total
const size_t ctestart=4*N; // first cte in vars, N in total
const size_t epsistart=5*N;// first epsi in vars, N in total
const size_t deltastart=6*N;// first delta in vars, N-1 in total
const size_t astart=6*N + 1*(N-1); //first a in vars, N-1 in total

// Cost factors
double cost_cte=100;//100
double cost_epsi=1000;//1000
double cost_v=1000;//100
double cost_delta=0;//1
double cost_a=0;//0
double cost_ddelta=0;//50
double cost_da=00;//200

// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
// Set speed for straight driving


class FG_eval {
public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }
  
  
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  
  // fg a vector of the cost constraints
  // vars is a vector of variable values (state & actuators)
  
  void operator()(ADvector& fg, const ADvector& vars) {
    
    // set the cost in fg[0]
    fg[0]=0;
    
    for (size_t i=0;i<N;i++) {
      
      AD<double> x = vars[xstart + i];
      //curvature calculation from https://www.math24.net/curvature-radius/
      AD<double> denom=1+CppAD::pow(coeffs[1] + 2 * coeffs[2] * x+3 * coeffs[3] * CppAD::pow(x, 2),2);
      AD<double> curve = CppAD::abs(2*coeffs[2]  + 6*coeffs[3] *x) /
      CppAD::pow(denom,1.5);
      AD<double> setspeed=v_set-CppAD::abs( curve*set_speed_factor);
      
      
      
      // Cost for offset from center
      fg[0] += cost_cte*CppAD::pow(vars[ctestart+i],2);
      // Cost for driving direction
      fg[0] += cost_epsi*CppAD::pow(vars[epsistart+i],2);
      //Cost for speed
      fg[0] += cost_v*CppAD::pow((vars[vstart+i]-setspeed)/v_set,2);
    }
    
    for (size_t i = 0; i < N - 1; i++) {
      //Cost for steering
      fg[0] += cost_delta*CppAD::pow(vars[deltastart + i], 2);
      //Cost for accelerating
      fg[0] += cost_a*CppAD::pow(vars[astart + i], 2);
    }
    for (size_t i = 0; i < N - 2; i++) {
      //Cost for too fast changing of steering angle
      fg[0] += cost_ddelta*CppAD::pow((vars[deltastart + i + 1] - vars[deltastart + i])/dt, 2);
      //Cost for too fast changing of acceleration
      fg[0] += cost_da*CppAD::pow((vars[astart + i + 1] - vars[astart + i])/dt, 2);
    }
    
    //set current values of the dynamic model in fg
    fg[1 + xstart] = vars[xstart];
    fg[1 + ystart] = vars[ystart];
    fg[1 + psistart] = vars[psistart];
    fg[1 + vstart] = vars[vstart];
    fg[1 + ctestart] = vars[ctestart];
    fg[1 + epsistart] = vars[epsistart];
    
    
    //calculate the values for fg for the following timesteps
    for (size_t t = 1; t < N; t++) {
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
      // include drag term proportional to v^2
      // the parameters have been determined by setting a constant acceleration
      // and see what the final constant speed is. In this case
      // 0.416 acceleration gives 48 Mph
      //
      // The factor 10 is also phenemological, to have the model
      // predict a realistic acceleration depending on the acceleration setting
      delta0= vars[deltastart];
      a0=1*(vars[astart]-0.416*v0*v0/48./48./MPH_mps/MPH_mps);
        
 
      
      // calculate the predicted y value of the road
      AD<double> fx0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      // calculate the predicted orientation of the road
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));// atan of the derivative
      
      // Dynamic model X1=f(X0,dt), so X1-f(X0,dt)==0
      fg[1 + xstart + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + ystart + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psistart + t]= psi1 - (psi0  + v0/Lf*delta0*dt);
      fg[1 + vstart + t] = v1 - (v0 + a0 * dt);
      
      // Offset from center and driving direction
      fg[1 + ctestart +t] = cte1-(fx0 - y0 + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsistart +t] = epsi1-(psides0-psi0 + v0 / Lf * delta0 * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  std::chrono::time_point<std::chrono::high_resolution_clock> start=std::chrono::high_resolution_clock::now();

  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  // Number of vars=6 * N + 2 * (N-1)
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
  double x=state[0];
  double y=state[1];
  double psi=state[2];
  double v=state[3]*MPH_mps;
  double cte=state[4];
  double epsi=state[5];
  double delta=-state[6]*deg2rad(25);
  double a=state[7]*10 -0.416*v*v/48./48./MPH_mps/MPH_mps;
  
  double fx = coeffs[0] + coeffs[1] * x + coeffs[2] * x*x + coeffs[3] * x*x*x;
  double psides = std::atan(coeffs[1] + 2 * coeffs[2] * x + 3 * coeffs[3] * x*x);
  
  vars[xstart]=x + v * std::cos(psi) * delay_t;//x
  vars[ystart]=y + v * std::sin(psi) * delay_t;//y
  vars[psistart]= psi  + v/Lf*delta*delay_t;//psi
  vars[vstart]=v + a * delay_t;//v
  vars[ctestart]=cte+(fx-y)+v * std::sin(epsi) * delay_t;//cte
  vars[epsistart]=epsi + (psi-psides)+v / Lf * delta * delay_t;//epsi
  
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
    vars_lowerbound[i]=-10;
    vars_upperbound[i]=10;
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
  
  constraints_lowerbound[xstart] = vars[xstart];
  constraints_lowerbound[ystart] = vars[ystart];
  constraints_lowerbound[psistart] = vars[psistart];
  constraints_lowerbound[vstart] = vars[vstart];
  constraints_lowerbound[ctestart] = vars[ctestart];
  constraints_lowerbound[epsistart] = vars[epsistart];
  
  constraints_upperbound[xstart] = vars[xstart];
  constraints_upperbound[ystart] = vars[ystart];
  constraints_upperbound[psistart] = vars[psistart];
  constraints_upperbound[vstart] = vars[vstart];
  constraints_upperbound[ctestart] = vars[ctestart];
  constraints_upperbound[epsistart] = vars[epsistart];
  
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
  
  // calculate the time spent since last solution
  std::chrono::time_point<std::chrono::high_resolution_clock> now=std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> tdelta = now-last;
  last=now;
  
  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  
  // Cost
  auto cost = solution.obj_value;
  
  x = solution.x[xstart];
  
  double denom=1+std::pow(coeffs[1] + 2 * coeffs[2] * x+3 * coeffs[3] * CppAD::pow(x, 2),2);
  double curve = std::abs(2*coeffs[2]  + 6*coeffs[3] *x) /
  std::pow(denom,1.5);
  double setspeed=v_set-std::abs( curve*set_speed_factor);
  
  
  //final costs:
  double c_cte= 0;
  double c_epsi= 0;
  double c_v= 0;
  double c_delta=0;
  double c_a= 0;
  double c_ddelta=0;
  double c_da= 0;
  
  for (size_t i=0;i<N;i++) {
    c_cte+= cost_cte*std::pow(solution.x[ctestart+i],2);
    c_epsi+= cost_epsi*std::pow(solution.x[epsistart+i],2);
    c_v+= cost_v*std::pow((solution.x[vstart+i]-setspeed)/v_set,2);
  }
  for (size_t i=0;i<N-1;i++) {
    c_delta+=cost_delta*std::pow(solution.x[deltastart+i], 2);
    c_a+= cost_a*std::pow(solution.x[astart+i], 2);
  }
  for (size_t i=0;i<N-2;i++){
    c_ddelta+=cost_ddelta*std::pow((solution.x[deltastart + 1+i] - solution.x[deltastart+i])/dt, 2);
    c_da+= cost_da*std::pow((solution.x[astart + 1+i] - solution.x[astart+i])/dt, 2);
  }
  double c_tot=c_cte+c_epsi+c_v+c_delta+c_a+c_ddelta+c_da;
  if (print_solution)std::printf("Solution: t=%7.2f c=%6.2f delta=%7.4f a=%7.4f set=%6.2f\n",tdelta.count(),cost, solution.x[deltastart], solution.x[astart], setspeed/MPH_mps);
  if (print_cost)std::printf("Costs: total=%7.2f cte=%7.2f epsi=%7.2f v=%7.2f delta=%7.2f a=%7.2f ddelta=%7.2e da=%7.2e\n",c_tot,c_cte,c_epsi,c_v,c_delta,c_a,c_ddelta,c_da);
  //  std::printf("%7.2f",tdelta.count());
  //  for (int i=0;i<N-1;i++) {
  //    std::printf(" %7.4e",solution.x[astart+i]- solution.x[astart+i]);
  //  };
  //  std::cout <<std::endl;
  
  
  
  //result[0]=steering angle
  //result[1]=acceleration
  //result[2..2+2N] x,y value pairs of predicted path
  vector<double> results(2+2*N);
  results[0]=-solution.x[deltastart]/deg2rad(25);
  results[1]=solution.x[astart]/10;
  for (i=0;i<N;i++) {
    results[2+2*i]=solution.x[xstart+i];
    results[2+2*i+1]=solution.x[ystart+i];
  }
  std::chrono::time_point<std::chrono::high_resolution_clock> end=std::chrono::high_resolution_clock::now();
   tdelta = end-start;
  if (tdelta.count()<calc_time) {
    int sleep=calc_time-tdelta.count();
//    std::cout <<sleep<<endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));

  }

  return results;
}
