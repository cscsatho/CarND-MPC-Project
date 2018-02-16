#include "MPC.h"
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.

class MPC;

class FG_eval
{
 public:
  // Fitted polynomial coefficients
  MPC& mpc_;

  FG_eval(MPC& mpc) : mpc_(mpc) { }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars)
  {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.


    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Reference State Cost
    // Define the cost related the reference state and
    // and anything you think may be beneficial.


// coefficients for the cost
/*
const double c_cte    = 1750;
const double c_epsi   = 8750;
const double c_v      = 1;
const double c_delta  = 3000;
const double c_a      = 1;
const double c_vd     = 400;
const double c_ddelta = 5000;
const double c_da     = 5; 
*/
/*
const double c_cte    = 750;
const double c_epsi   = 750;
const double c_v      = 1;
const double c_delta  = 1;
const double c_a      = 1;
const double c_ddelta = 50;
const double c_da     = 5; 
const double c_vd     = 400;
*/
/*
const double c_cte    = 500;
const double c_epsi   = 1000;
const double c_v      = 1;
const double c_delta  = 1000;
const double c_a      = 1;
const double c_ddelta = 20000;
const double c_da     = 1; 
*/
const double c_cte    = 200;
const double c_epsi   = 500;
const double c_v      = 1;
const double c_delta  = 500;
const double c_a      = 1;
const double c_ddelta = 9500;
const double c_da     = 1; 


    // The part of the cost based on the reference state.
    for (int t = 0; t < mpc_.N; ++t)
    { // FIXME - refactor
      fg[0] += c_cte * CppAD::pow(vars[mpc_.Cidx + t], 2);
      fg[0] += c_epsi * CppAD::pow(vars[mpc_.Eidx + t], 2);
      fg[0] += c_v * CppAD::pow(vars[mpc_.Vidx + t] - mpc_.V_REFERENCE, 2);
//      fg[0] += CppAD::pow(pow(vars[mpc_.Xidx + t] - xEnd, 2) + pow(vars[mpc_.Yidx + t] - yEnd, 2), 0.5);
      if (t < mpc_.N - 1)
      {
        fg[0] += c_delta * CppAD::pow(vars[mpc_.Didx + t], 2);
        fg[0] += c_a * CppAD::pow(vars[mpc_.Aidx + t], 2);
//        fg[0] += c_vd * CppAD::pow(vars[mpc_.Didx + t] * vars[mpc_.Vidx + t], 2); // FIXME
        if (t < mpc_.N - 2)
        {
          fg[0] += c_ddelta * CppAD::pow(vars[mpc_.Didx + t + 1] - vars[mpc_.Didx + t], 2);
          fg[0] += c_da * CppAD::pow(vars[mpc_.Aidx + t + 1] - vars[mpc_.Aidx + t], 2);
        }
      }
    }

    // Setting up model constraints

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + mpc_.Xidx] = vars[mpc_.Xidx];
    fg[1 + mpc_.Yidx] = vars[mpc_.Yidx];
    fg[1 + mpc_.Pidx] = vars[mpc_.Pidx];
    fg[1 + mpc_.Vidx] = vars[mpc_.Vidx];
    fg[1 + mpc_.Cidx] = vars[mpc_.Cidx];
    fg[1 + mpc_.Eidx] = vars[mpc_.Eidx];

    // The rest of the constraints
    for (int t = 1; t < mpc_.N; ++t)
    {
      // The state at time t+1 .
      AD<double> x1      = vars[mpc_.Xidx + t];
      AD<double> y1      = vars[mpc_.Yidx + t];
      AD<double> psi1    = vars[mpc_.Pidx + t];
      AD<double> v1      = vars[mpc_.Vidx + t];
      AD<double> cte1    = vars[mpc_.Cidx + t];
      AD<double> epsi1   = vars[mpc_.Eidx + t];

      // The state at time t.
      AD<double> x0      = vars[mpc_.Xidx + t - 1];
      AD<double> y0      = vars[mpc_.Yidx + t - 1];
      AD<double> psi0    = vars[mpc_.Pidx + t - 1];
      AD<double> v0      = vars[mpc_.Vidx + t - 1];
      AD<double> cte0    = vars[mpc_.Cidx + t - 1];
      AD<double> epsi0   = vars[mpc_.Eidx + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0  = vars[mpc_.Didx + t - 1];
      AD<double> a0      = vars[mpc_.Aidx + t - 1];

      AD<double> f0      = mpc_.polyevalAD(x0);
      AD<double> psides0 = mpc_.polyevalPrimeAD(x0);

//      if (t > 1) {   // use previous actuations (to account for latency)
//        delta0 = vars[mpc_.Didx + t - 2];  // FIXME
//        a0 = vars[mpc_.Aidx + t - 2];
//      }

      fg[1 + mpc_.Xidx + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * mpc_.DT);
      fg[1 + mpc_.Yidx + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * mpc_.DT);
      fg[1 + mpc_.Pidx + t] = psi1 - (psi0 + v0 / MPC::Lf * delta0 * mpc_.DT);
      fg[1 + mpc_.Vidx + t] = v1 - (v0 + a0 * mpc_.DT);
      //fg[1 + mpc_.Cidx + t] = cte1 - (cte0 + v0 * CppAD::sin(epsi0) * mpc_.DT);
      //fg[1 + mpc_.Eidx + t] = epsi1 - (epsi0 - v0 / MPC::Lf * delta0 * mpc_.DT); // FIXME -?
      fg[1 + mpc_.Cidx + t] = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * mpc_.DT);
      fg[1 + mpc_.Eidx + t] = epsi1 - (psi0 - psides0 + v0 / MPC::Lf * delta0 * mpc_.DT);
    }
  }
};

//
// MPC class definition implementation.
//
const double MPC::Lf = 2.67;

MPC::MPC(const int n, const double& dt, const double& lat, const int order,
    const double& steer_min, const double& steer_max,
    const double& accel_min, const double& accel_max,
    const double& v_reference)
  : N(n), DT(dt), LATENCY(lat), POLYORDER(order),
    Xidx(0), Yidx(Xidx + N),
    Pidx(Yidx + N), Vidx(Pidx + N),
    Cidx(Vidx + N), Eidx(Cidx + N),
    Didx(Eidx + N), Aidx(Didx + N - 1),
    state_(6), coeffs_(order), //state_next_(6), initialized_(false),
    STEER_MIN(steer_min), STEER_MAX(steer_max),
    ACCEL_MIN(accel_min), ACCEL_MAX(accel_max),
    V_REFERENCE(v_reference),
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    // Uncomment this if you'd like more print information
    SOLVER_OPTIONS("Integer print_level  0\n"
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
                   "Sparse  true         forward\n"
                   "Sparse  true         reverse\n"
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
                   "Numeric max_cpu_time 0.5\n")
{
}

void MPC::preprocess(vector<double>& ptsx, vector<double>& ptsy,
                     const double& px, const double& py, const double& psi, const double& v,
                     const double& delta0, const double& a0)
{
  ptsx_car_.resize(ptsx.size());
  ptsy_car_.resize(ptsy.size());

  assert (ptsx.size() == ptsy.size());

  for (unsigned short n = 0; n < ptsx.size(); ++n)
  {
    ptsx_car_[n] = (ptsx[n] - px) * cos(psi) + (ptsy[n] - py) * sin(psi);
    ptsy_car_[n] = (px - ptsx[n]) * sin(psi) + (ptsy[n] - py) * cos(psi);
  } // px, py, and psi was adjusted to be 0

  // Recall the equations for the model:
  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
  // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
  // v_[t+1] = v[t] + a[t] * dt
  // cte[t+1] =  y[t] - f(x[t]) + v[t] * sin(epsi[t]) * dt
  // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

  static const double PX_CAR = 0.0;
  static const double PY_CAR = 0.0;
  static const double PSI_CAR = 0.0;

  coeffs_ = polyfit();

  const double cte = coeffs_[0] - PY_CAR; // all other coeffs would become 0
  const double epsi = PSI_CAR - atan(coeffs_[1]); // adjusted psi is also 0
  const double psi_corr = v * delta0 / Lf * LATENCY;

  // Set the initial state values incorporating latency
//  state_ = state_next_;
//  state_next_ << PX_CAR, PY_CAR, PSI_CAR, v, cte, epsi;
  state_ << PX_CAR + v * LATENCY,
            PY_CAR,
            PSI_CAR + psi_corr,
            v + a0 * LATENCY,
            cte + v * sin(epsi) * LATENCY,
            epsi + psi_corr;
//  if (initialized_) return true;
//
//  initialized_ = true;
//  return false;

}

vector<double> MPC::solve()
{
  bool ok = true;
  size_t i;

  double x    = state_[0];
  double y    = state_[1];
  double psi  = state_[2];
  double v    = state_[3];
  double cte  = state_[4];
  double epsi = state_[5];

  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (i = 0; i < n_vars; ++i) vars[i] = 0;

  // Set the initial variable values
  vars[Xidx]    = x;
  vars[Yidx]    = y;
  vars[Pidx]  = psi;
  vars[Vidx]    = v;
  vars[Cidx]  = cte;
  vars[Eidx] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set lower and upper limits for variables.

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (i = 0; i < Didx; ++i)
  { // FIXME = check
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] =  1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (i = Didx; i < Aidx; ++i)
  { // FIXME = check
    vars_lowerbound[i] = STEER_MIN;
    vars_upperbound[i] = STEER_MAX;
  }

  // Acceleration/decceleration upper and lower limits.
  for (i = Aidx; i < n_vars; ++i)
  { // FIXME = check
    vars_lowerbound[i] = ACCEL_MIN;
    vars_upperbound[i] = ACCEL_MAX;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (i = 0; i < n_constraints; ++i)
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[Xidx] = x;
  constraints_lowerbound[Yidx] = y;
  constraints_lowerbound[Pidx] = psi;
  constraints_lowerbound[Vidx] = v;
  constraints_lowerbound[Cidx] = cte;
  constraints_lowerbound[Eidx] = epsi;

  constraints_upperbound[Xidx] = x;
  constraints_upperbound[Yidx] = y;
  constraints_upperbound[Pidx] = psi;
  constraints_upperbound[Vidx] = v;
  constraints_upperbound[Cidx] = cte;
  constraints_upperbound[Eidx] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(*this);

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      SOLVER_OPTIONS, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> res(2 + (N - 1) * 2);
  res[0] = solution.x[Didx];
  res[1] = solution.x[Aidx];
  for (unsigned short n = 0; n <= N - 2; ++n)
  {
    res[2 * n + 2] = solution.x[Xidx + 1 + n];
    res[2 * n + 3] = solution.x[Yidx + 1 + n];
  }
//  return { solution.x[Xidx + 1],   solution.x[Yidx + 1],
//           solution.x[Pidx + 1], solution.x[Vidx + 1],
//           solution.x[Cidx + 1], solution.x[Eidx + 1],
//           solution.x[Didx],   solution.x[Aidx] };
  return res;
}

// Evaluate a polynomial.
double MPC::polyeval(const double& x) const
{
  double result = 0.0;
  for (int i = 0; i < coeffs_.size(); ++i)
    if (i > 1)
      result += coeffs_[i] * pow(x, i);
    else if (i == 1)
      result += coeffs_[i] * x;
    else
      result += coeffs_[i];
  return result;
}

double MPC::polyevalPrime(const double& x) const
{
  double result = 0.0;
  for (int i = 1; i < coeffs_.size(); ++i)
    if (i > 2)
      result += i * coeffs_[i] * pow(x, i - 1);
    else if (i == 2)
      result += 2 * coeffs_[i] * x;
    else
      result += coeffs_[i];
  return result;
}

AD<double> MPC::polyevalAD(const AD<double>& x) const
{
  AD<double> result = 0.0;
  for (int i = 0; i < coeffs_.size(); ++i)
    if (i > 1)
      result += coeffs_[i] * CppAD::pow(x, i);
    else if (i == 1)
      result += coeffs_[i] * x;
    else
      result += coeffs_[i];
  return result;
}

AD<double> MPC::polyevalPrimeAD(const AD<double>& x) const
{
  AD<double> result = 0.0;
  for (int i = 1; i < coeffs_.size(); ++i)
    if (i > 2)
      result += i * coeffs_[i] * CppAD::pow(x, i - 1);
    else if (i == 2)
      result += 2 * coeffs_[i] * x;
    else
      result += coeffs_[i];
  return result;
}

// Fit a polynomial. Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd MPC::polyfit()
{
  assert(ptsx_car_.size() == ptsy_car_.size());
  assert(POLYORDER >= 1 && POLYORDER <= (int)ptsx_car_.size() - 1);

  Eigen::MatrixXd A(ptsx_car_.size(), POLYORDER + 1);

  for (int i = 0; i < (int)ptsx_car_.size(); i++)
    A(i, 0) = 1.0;

  for (int j = 0; j < (int)ptsx_car_.size(); j++)
    for (int i = 0; i < POLYORDER; i++)
      A(j, i + 1) = A(j, i) * ptsx_car_[j];

  const Eigen::VectorXd ptsy_car_E = Eigen::Map<Eigen::VectorXd>(ptsy_car_.data(), ptsy_car_.size());
  //auto ptsy_car_E = Eigen::Map<Eigen::VectorXd>(ptsy_car_.data(), ptsy_car_.size());

  auto Q = A.householderQr();
  auto result = Q.solve(ptsy_car_E);
  return result;
}
