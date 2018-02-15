#include "MPC.h"
#include <cppad/cppad.hpp>
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
  const Eigen::VectorXd coeffs_;
  MPC& mpc_;

  FG_eval(const Eigen::VectorXd& coeffs, MPC& mpc) : coeffs_(coeffs), mpc_(mpc) { }

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

    // The part of the cost based on the reference state.
    for (int t = 0; t < mpc_.N_; ++t)
    {
      fg[0] += CppAD::pow(vars[mpc_.cte_start_ + t], 2);
      fg[0] += CppAD::pow(vars[mpc_.epsi_start_ + t], 2);
//      fg[0] += CppAD::pow(vars[mpc_.v_start_ + t] - ref_v, 2); // FIXME set ref_velocity
//      fg[0] += CppAD::pow(pow(vars[mpc_.x_start_ + t] - xEnd, 2) + pow(vars[mpc_.y_start_ + t] - yEnd, 2), 0.5);
      if (t < mpc_.N_ - 1)
      {
        fg[0] += CppAD::pow(vars[mpc_.delta_start_ + t], 2);
        fg[0] += CppAD::pow(vars[mpc_.a_start_ + t], 2);
        if (t < mpc_.N_ - 2)
        {
          fg[0] += CppAD::pow(vars[mpc_.delta_start_ + t + 1] - vars[mpc_.delta_start_ + t], 2);
          fg[0] += CppAD::pow(vars[mpc_.a_start_ + t + 1] - vars[mpc_.a_start_ + t], 2);
        }
      }
    }

    // Setting up model constraints

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + mpc_.x_start_]    = vars[mpc_.x_start_];
    fg[1 + mpc_.y_start_]    = vars[mpc_.y_start_];
    fg[1 + mpc_.psi_start_]  = vars[mpc_.psi_start_];
    fg[1 + mpc_.v_start_]    = vars[mpc_.v_start_];
    fg[1 + mpc_.cte_start_]  = vars[mpc_.cte_start_];
    fg[1 + mpc_.epsi_start_] = vars[mpc_.epsi_start_];

    // The rest of the constraints
    for (int t = 1; t < mpc_.N_; ++t)
    {
      // The state at time t+1 .
      AD<double> x1     = vars[mpc_.x_start_ + t];
      AD<double> y1     = vars[mpc_.y_start_ + t];
      AD<double> psi1   = vars[mpc_.psi_start_ + t];
      AD<double> v1     = vars[mpc_.v_start_ + t];
      AD<double> cte1   = vars[mpc_.cte_start_ + t];
      AD<double> epsi1  = vars[mpc_.epsi_start_ + t];

      // The state at time t.
      AD<double> x0     = vars[mpc_.x_start_ + t - 1];
      AD<double> y0     = vars[mpc_.y_start_ + t - 1];
      AD<double> psi0   = vars[mpc_.psi_start_ + t - 1];
      AD<double> v0     = vars[mpc_.v_start_ + t - 1];
      AD<double> cte0   = vars[mpc_.cte_start_ + t - 1];
      AD<double> epsi0  = vars[mpc_.epsi_start_ + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[mpc_.delta_start_ + t - 1];
      AD<double> a0     = vars[mpc_.a_start_ + t - 1];

      fg[1 + mpc_.x_start_ + t]    = x1 - (x0 + v0 * CppAD::cos(psi0) * mpc_.dt_);
      fg[1 + mpc_.y_start_ + t]    = y1 - (y0 + v0 * CppAD::sin(psi0) * mpc_.dt_);
      fg[1 + mpc_.psi_start_ + t]  = psi1 - (psi0 + v0 / MPC::Lf_ * delta0 * mpc_.dt_);
      fg[1 + mpc_.v_start_ + t]    = v1 - (v0 + a0 * mpc_.dt_);
      fg[1 + mpc_.cte_start_ + t]  = cte1 - (cte0 + v0 * CppAD::sin(epsi0) * mpc_.dt_);
      fg[1 + mpc_.epsi_start_ + t] = epsi1 - (epsi0 + v0 / MPC::Lf_ * delta0 * mpc_.dt_);
    }
  }
};

//
// MPC class definition implementation.
//
const double MPC::Lf_ = 2.67;

MPC::MPC(const int N, const double dt, const int order,
    const double steer_min, const double steer_max,
    const double accel_min, const double accel_max)
  : N_(N), dt_(dt), polyorder_(order),
    x_start_(0), y_start_(x_start_ + N_),
    psi_start_(y_start_ + N_), v_start_(psi_start_ + N_),
    cte_start_(v_start_ + N_), epsi_start_(cte_start_ + N_),
    delta_start_(epsi_start_ + N_), a_start_(delta_start_ + N_ - 1),
    state_(6), coeffs_(order), state_next_(6), initialized_(false),
    STEER_MIN(steer_min), STEER_MAX(steer_max),
    ACCEL_MIN(accel_min), ACCEL_MAX(accel_max)
{
  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  // Uncomment this if you'd like more print information
  solver_options_ += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  solver_options_ += "Sparse  true        forward\n";
  solver_options_ += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  solver_options_ += "Numeric max_cpu_time          0.5\n";
}

bool MPC::preprocess(vector<double>& ptsx, vector<double>& ptsy,
                     const double& px, const double& py, const double& psi, const double& v)
{
  coeffs_ = Polyfit(ptsx, ptsy, polyorder_);

  double cte = py - polyeval(px);
  double epsi = psi - atan(polyevalPrime(px));

  state_ = state_next_;
  state_next_ << px, py, psi, v, cte, epsi;
  if (initialized_) return true;

  initialized_ = true;
  return false;
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
  size_t n_vars = N_ * 6 + (N_ - 1) * 2;
  // Set the number of constraints
  size_t n_constraints = N_ * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (i = 0; i < n_vars; ++i) vars[i] = 0;

  // Set the initial variable values
  vars[x_start_]    = x;
  vars[y_start_]    = y;
  vars[psi_start_]  = psi;
  vars[v_start_]    = v;
  vars[cte_start_]  = cte;
  vars[epsi_start_] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set lower and upper limits for variables.

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (i = 0; i < delta_start_; ++i)
  { // FIXME = check
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (i = delta_start_; i < a_start_; ++i)
  { // FIXME = check
    vars_lowerbound[i] = STEER_MIN;
    vars_upperbound[i] = STEER_MAX;
  }

  // Acceleration/decceleration upper and lower limits.
  for (i = a_start_; i < n_vars; ++i)
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

  constraints_lowerbound[x_start_] = x;
  constraints_lowerbound[y_start_] = y;
  constraints_lowerbound[psi_start_] = psi;
  constraints_lowerbound[v_start_] = v;
  constraints_lowerbound[cte_start_] = cte;
  constraints_lowerbound[epsi_start_] = epsi;

  constraints_upperbound[x_start_] = x;
  constraints_upperbound[y_start_] = y;
  constraints_upperbound[psi_start_] = psi;
  constraints_upperbound[v_start_] = v;
  constraints_upperbound[cte_start_] = cte;
  constraints_upperbound[epsi_start_] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs_, *this);

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      solver_options_, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
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
  return { solution.x[x_start_ + 1],   solution.x[y_start_ + 1],
           solution.x[psi_start_ + 1], solution.x[v_start_ + 1],
           solution.x[cte_start_ + 1], solution.x[epsi_start_ + 1],
           solution.x[delta_start_],   solution.x[a_start_] };
}

// Evaluate a polynomial.
double MPC::polyeval(const double& x) const
{
  double result = 0.0;
  for (int i = 0; i < coeffs_.size(); ++i)
    result += coeffs_[i] * pow(x, i);
  return result;
}

double MPC::polyevalPrime(const double& x) const
{
  double result = 0.0;
  for (int i = 1; i < coeffs_.size(); ++i)
    result += i * coeffs_[i] * pow(x, i - 1);
  return result;
}

//static AD<double> Polyeval(const FG_eval::Dvector& coeffs, const double& x)
//{
//  AD<double> result = 0.0;
//  for (int i = 0; i < coeffs.size(); ++i)
//    result += coeffs[i] * CppAD::pow(x, i);
//  return result;
//}

// Fit a polynomial. Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd MPC::Polyfit(vector<double>& xvals, vector<double>& yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= (int)xvals.size() - 1);

  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < (int)xvals.size(); i++)
    A(i, 0) = 1.0;

  for (int j = 0; j < (int)xvals.size(); j++)
    for (int i = 0; i < order; i++)
      A(j, i + 1) = A(j, i) * xvals[j];

  const Eigen::VectorXd yvalsE = Eigen::Map<Eigen::VectorXd>(yvals.data(), yvals.size());
  //auto yvalsE = Eigen::Map<Eigen::VectorXd>(yvals.data(), yvals.size());

  auto Q = A.householderQr();
  auto result = Q.solve(yvalsE);
  return result;
}
