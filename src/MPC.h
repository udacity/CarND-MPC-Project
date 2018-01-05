#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <ctime>
#include <ratio>
#include <chrono>

// number of timesteps delay. Timestep assumed to be 100 ms.
const int delay_steps=1;
// Number of timesteps in model
const size_t N = 10;
// size of timestep in [s]
const double dt = 0.1;

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
