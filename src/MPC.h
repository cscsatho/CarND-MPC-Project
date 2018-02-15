#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC
{
 public:
  MPC(const int N, const double dt, const int order,
      const double steer_min, const double steer_max,
      const double accel_min, const double accel_max);
  virtual ~MPC() = default;

  bool preprocess(vector<double>& ptsx, vector<double>& ptsy,
                  const double& px, const double& py, const double& psi, const double& v);

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> solve();

  //static Eigen::VectorXd Polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, int order);
  static Eigen::VectorXd Polyfit(vector<double>& xvals, vector<double>& yvals, int order);

  double polyeval(const double& x) const;
  double polyevalPrime(const double& x) const;

  const int N_;
  const double dt_; // that equals the latency by design
  const int polyorder_;
  static const double Lf_;

  const size_t x_start_;
  const size_t y_start_;
  const size_t psi_start_;
  const size_t v_start_;
  const size_t cte_start_;
  const size_t epsi_start_;
  const size_t delta_start_;
  const size_t a_start_;

  Eigen::VectorXd state_;
  Eigen::VectorXd coeffs_;
  Eigen::VectorXd state_next_;

  bool initialized_;

  const double STEER_MIN;
  const double STEER_MAX;
  const double ACCEL_MIN;
  const double ACCEL_MAX;

  std::string solver_options_;
};

#endif /* MPC_H */
