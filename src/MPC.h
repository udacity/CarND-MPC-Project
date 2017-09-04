#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  vector<double> ptsx;
  vector<double> ptsy;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs, double target_x, double target_y);
};

#endif /* MPC_H */
