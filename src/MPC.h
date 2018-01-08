#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <ctime>
#include <ratio>
#include <chrono>

const double MPH_mps=0.44704;

// number of timesteps delay. Timestep assumed to be 100 ms.
const int delay=00;//ms
const int calc_time=30;//average calculation time in ms
const double delay_t=1E-3*delay+1E-3*calc_time;
// Number of timesteps in model
const size_t N = 10;
// size of timestep in [s]
const double dt = 0.1;

const double v_set=100*MPH_mps;
// factor to reduce the set speed, based on curvature of road
//setspeed=v_set-set_speed_factor*curvature
const double set_speed_factor=800;


const bool print_solution=1;
const bool print_cost=0;

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
private:
  std::chrono::time_point<std::chrono::high_resolution_clock> last=std::chrono::high_resolution_clock::now();

};

#endif /* MPC_H */
