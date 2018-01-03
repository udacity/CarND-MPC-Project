//
//  helper_functions.hpp
//  MPC
//
//  Created by Igor Passchier on 31/12/2017.
//

#ifndef helper_functions_hpp
#define helper_functions_hpp

#include <stdio.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }


// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) ;

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order);

std::vector<double> unityToCar(double x, double y, double psi, double car_x, double car_y, double car_psi);

void unityToCar(std::vector<double> &car_x_vals, std::vector<double> &car_y_vals,
                std::vector<double> &unity_x_vals, std::vector<double> &unity_y_vals, double px, double py, double psi);

inline double limit_range(double x) {
  double ret=x;
  while (ret<0)ret+=2*M_PI;
  while(ret>=2*M_PI)ret-=2*M_PI;
  return ret;
}
#endif /* helper_functions_hpp */
