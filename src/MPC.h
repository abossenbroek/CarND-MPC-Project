#ifndef MPC_H
#define MPC_H

#include <vector>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
private:
  double a_;
  double delta_;

public:
  MPC();

  virtual ~MPC();

  static const int states_in_latency;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  double steeringValue() {
    return -delta_ / (25. * M_PI / 180.);
  }

  double throttleValue() {
    return a_;
  }

  vector<double> pred_path_x_;
  vector<double> pred_path_y_;
};

#endif /* MPC_H */
