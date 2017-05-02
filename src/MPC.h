#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
