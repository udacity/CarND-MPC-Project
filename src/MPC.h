#ifndef MPC_H
#define MPC_H

#include <vector>

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state.
  // Return the next state, inputs and cost.
  // NOTE: You may change the function signature it's helpful.
  tuple<vector<double>, vector<double>, double> Solve(vector<double> x0);
};

#endif /* MPC_H */
