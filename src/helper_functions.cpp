//
//  helper_functions.cpp
//  MPC
//
//  Created by Igor Passchier on 31/12/2017.
//

#include "helper_functions.hpp"
#include <vector>

using namespace std;
// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);
  
  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }
  
  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }
  
  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

vector<double> unityToCar(double x, double y, double psi, double car_x, double car_y, double car_psi) {
  std::vector<double> ret(3);

  ret[0]= (x-car_x)*cos(car_psi)+(y-car_y)*sin(car_psi);
  ret[1]=-(x-car_x)*sin(car_psi)+(y-car_y)*cos(car_psi);
  ret[2]=limit_range(psi-car_psi);
  return ret;
}

void unityToCar(vector<double> &car_x_vals, vector<double> &car_y_vals,
                      vector<double> &unity_x_vals, vector<double> &unity_y_vals, double px, double py, double psi) {
  for(int i=0;i<unity_x_vals.size();++i) {
    double wx=unity_x_vals[i];
    double wy=unity_y_vals[i];
    vector<double> point=unityToCar(wx, wy, 0, px, py, psi);
    car_x_vals.push_back(point[0]);
    car_y_vals.push_back(point[1]);
    
    //cout << wx <<", "<<wy <<": "<<point[0]<<", "<<point[1]<<std::endl;
  }
}
