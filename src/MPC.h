#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>

using namespace std;

class MPC
{
 public:
  MPC(const int N, const double& dt, const double& lat, const int order,
      const double& steer_min, const double& steer_max,
      const double& accel_min, const double& accel_max,
      const double& v_reference);
  virtual ~MPC() = default;

  void preprocess(vector<double>& ptsx, vector<double>& ptsy,
                  const double& px, const double& py, const double& psi, const double& v,
                  const double& delta0, const double& a0);

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> solve();

  //static Eigen::VectorXd Polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, int order);
  Eigen::VectorXd polyfit();

  double polyeval(const double& x) const;
  double polyevalPrime(const double& x) const;
  CppAD::AD<double> polyevalAD(const CppAD::AD<double>& x) const;
  CppAD::AD<double> polyevalPrimeAD(const CppAD::AD<double>& x) const;

  const int N;
  const double DT;
  const double LATENCY;
  const int POLYORDER;
  static const double Lf;

  vector<double> ptsx_car_; // waypoints-x in car coordinates
  vector<double> ptsy_car_; // waypoints-y in car coordinates

  const size_t Xidx; // start index of x coord 
  const size_t Yidx; // start index of y coord 
  const size_t Pidx; // start index of psi 
  const size_t Vidx; // start index of velocity 
  const size_t Cidx; // start index of cte
  const size_t Eidx; // start index of epsi
  const size_t Didx; // start index of delta
  const size_t Aidx; // start index of acceleration

  Eigen::VectorXd state_;
  Eigen::VectorXd coeffs_;
//  Eigen::VectorXd state_next_;

//  bool initialized_;

  const double STEER_MIN;
  const double STEER_MAX;
  const double ACCEL_MIN;
  const double ACCEL_MAX;

  const double V_REFERENCE;

  const std::string SOLVER_OPTIONS;
};

#endif /* MPC_H */
