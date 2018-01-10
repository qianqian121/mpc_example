#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

const double dt = 0.1;
const double Lf = 2.67;

int main() {
  // MPC is initialized here!
  MPC mpc;

          // j[1] is the data JSON object
          vector<double> ptsx = {0, 10, 20, 30, 40, 50};
          vector<double> ptsy = {0, 0, 0, 0, 0, 0};
          double x = 0;
          double y = 0;
          double psi = 0;
          double v = 5;

          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          for (int i = 0; i < ptsx.size(); i++ )
          {
            //rotate the difference between the waypoints and the car position by -psi.
            double diff_x = ptsx[i]-x;
            double diff_y = ptsy[i]-y;
            ptsx[i] = (diff_x *cos(-psi)-diff_y*sin(-psi));
            ptsy[i] = (diff_x *sin(-psi)+diff_y*cos(-psi));
          }

          double steer_value = 0.0;
          double throttle_value = 0.0;

          // The polynomial is fitted to a curve line so a polynomial with
          // order 3 is sufficient.
          Eigen::Map<Eigen::VectorXd> ptsx_ev(ptsx.data(), ptsx.size());
          Eigen::Map<Eigen::VectorXd> ptsy_ev(ptsy.data(),ptsy.size());
          auto coeffs = polyfit(ptsx_ev, ptsy_ev, 3);

          // The cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y.
          double cte = polyeval(coeffs, 0);
          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          double epsi = -atan(coeffs[1]);

          //transition from t to t+1 (time step is delta_t = 0.1)
          double next_x = v * dt;
          double next_y = 0;
          double next_psi = -v * steer_value / Lf * dt;
          double next_v = v + throttle_value * dt;
          double next_cte = cte + v * sin(epsi) * dt;
          double next_epsi = epsi-v * steer_value /Lf * dt;

          Eigen::VectorXd state(6);
          state << next_x, next_y, next_psi, next_v, next_cte, next_epsi;

          auto vars = mpc.Solve(state, coeffs);

          steer_value = vars[vars.size() - 2];
          throttle_value = vars[vars.size() - 1];
//          throttle_value = 0.1;
}
