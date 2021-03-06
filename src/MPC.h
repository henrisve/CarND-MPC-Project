#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  bool twiddle(double speed,int x, int y, double cte);
  double best_speed=0;
  int counter=0;
  double total_speed=0;
  double max_speed=0;
  int parameterNo = 0;
  bool twiddleCheckNeg = true;
  int old_x=0;
  int old_y=0;
  int best_times=0;
  //std::vector<double> mpc_params;
  //std::vector<double> twiddleDP;
};

#endif /* MPC_H */
