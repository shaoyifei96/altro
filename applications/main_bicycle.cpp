// Most Basic Test of ALTRO
// #include "altro/altro.hpp"
// #include "fmt/core.h"
// #include "Eigen/Dense"

// int main() {

// }

#include "Eigen/Dense"
#include "altro/altro.hpp"
#include "fmt/core.h"
#include "../test/test_utils.hpp"
#include <chrono>
#include <fstream>
#include <filesystem>
#include <string>
#include <iostream>
#include "nlohmann/json.hpp"
// #include "gtest/gtest.h"
namespace fs = std::filesystem;
using json = nlohmann::json;


using Eigen::MatrixXd;
using Eigen::MatrixXd;
// using Eigen::Vector;

using namespace altro;
using Vector = Eigen::Matrix<a_float, Eigen::Dynamic, 1>;
using Matrix = Eigen::Matrix<a_float, Eigen::Dynamic, Eigen::Dynamic>;
using Eigen::VectorXi;

// const std::string ALTRO_TEST_DIR ="/home/yifei/Documents/optimal_ctrl/try_altro";


class BicycleMPC {
 public:
  BicycleMPC() : solver(N) {}
  void SetUp() {
    ErrorCodes err;
    // Objective

    Qd = Vector::Constant(n, 1e-2);
    Rd = Vector::Constant(m, 1e-3);
    Qdf = Vector::Constant(n, 1e1);

    // Dynamics
    auto model_ptr = std::make_shared<BicycleModel>();
    ContinuousDynamicsFunction dyn0 = [model_ptr](double *x_dot, const double *x, const double *u) {
      model_ptr->Dynamics(x_dot, x, u);
    };
    ContinuousDynamicsJacobian jac0 = [model_ptr](double *jac, const double *x, const double *u) {
      model_ptr->Jacobian(jac, x, u);
    };
    dyn = MidpointDynamics(n, m, dyn0);
    jac = MidpointJacobian(n, m, dyn0, jac0);


    // Dimension and Time step
    err = solver.SetDimension(n, m);

    // Dynamics
    err = solver.SetExplicitDynamics(dyn, jac);

    // Read Reference Trajectory
    int N_ref;
    float t_ref;
    ReadScottyTrajectory(&N_ref, &t_ref, &x_ref, &u_ref);
    fmt::print("Trajectory Read!\n");
    // for (int k = 0; k <= N; ++k) {
    //   std::cout << "x_ref[" << k << "] = " << x_ref[k].transpose() << std::endl;
    //   std::cout << "u_ref[" << k << "] = " << u_ref[k].transpose() << std::endl;
    // }
    a_float * Qd_data = this->Qd.data();
    std::cout << "Qd = " << Qd_data[0] << std::endl;
    std::cout << "N_ref" << N_ref << std::endl;

    // Set time step equal to the reference trajectory
    h = t_ref / static_cast<double>(N_ref);
    err = solver.SetTimeStep(h);

    fmt::print("Set TimeStep Finished!\n");
    PrintErrorCode(err);

    // Set Tracking Cost Function
    for (int k = 0; k <= N; ++k) {
      // std::cout << k << std::endl;
      err = solver.SetLQRCost(n, m, Qd.data(), Rd.data(), x_ref.at(k).data(), u_ref.at(k).data(), k);
      // PrintErrorCode(err);
      // fmt::print("Print error\n");
    }
    fmt::print("Set Cost Finished!\n");

    // Constraints
    auto steering_angle_con = [](a_float *c, const a_float *x, const a_float *u) {
      (void)u;
      a_float delta_max = 60 * M_PI / 180.0;
      c[0] = x[3] - delta_max;
      c[1] = -delta_max - x[3];
    };
    auto steering_angle_jac = [](a_float *jac, const a_float *x, const a_float *u) {
      (void)x;
      (void)u;
      Eigen::Map<Eigen::Matrix<a_float, 2, 6>> J(jac);
      J.setZero();
      J(0, 3) = 1.0;
      J(1, 3) = -1.0;
    };
    err = solver.SetConstraint(steering_angle_con, steering_angle_jac, 2, ConstraintType::INEQUALITY,
                               "steering angle bound", 0, N + 1);
    fmt::print("Set Constraints Finished!\n");

    // Initial State
    err = solver.SetInitialState(x_ref[0].data(), n);

    // Initialize Solver
    err = solver.Initialize();
    fmt::print("Solver Initialized!\n");

    // Set Initial Trajectory
    a_float average_speed = u_ref[0][0];
    u0 = Vector::Zero(m);
    u0 << average_speed, 0.0;
    err = solver.SetInput(u0.data(), m);
    for (int k = 0; k <= N; ++k) {
      solver.SetState(x_ref.at(k).data(), n, k);
    }
  }
  protected:
    const int n = BicycleModel::NumStates;
    const int m = BicycleModel::NumInputs;
    const int N = 30;
    float h;

    Vector Qd;
    Vector Rd;
    Vector Qdf;

    ExplicitDynamicsFunction dyn;
    ExplicitDynamicsJacobian jac;

    ALTROSolver solver;

    // Reference Trajectory (the "Scotty Dog")
    std::vector<Eigen::Vector4d> x_ref;
    std::vector<Eigen::Vector2d> u_ref;
    Vector u0;

  public:
    void eg1() {
      a_float cost_initial = solver.CalcCost();
      fmt::print("Initial cost = {}\n", cost_initial);

      solver.OpenLoopRollout();
      cost_initial = solver.CalcCost();
      fmt::print("Initial cost = {}\n", cost_initial);

      // Solve
      AltroOptions opts;
      opts.verbose = Verbosity::Inner;
      opts.iterations_max = 80;
      opts.use_backtracking_linesearch = true;
      solver.SetOptions(opts);
      SolveStatus status = solver.Solve();
    }
    void eg2() {
      fmt::print("#############################################\n");
      fmt::print("                 MPC Solve\n");
      fmt::print("#############################################\n");
      // MPC Setup
      int Nsim = 200;
      int mpc_iter = 0;
      std::vector<Vector> x_sim;
      std::vector<Vector> u_sim;
      std::vector<int> solve_iters;
      std::vector<double> tracking_error;
      x_sim.reserve(Nsim + 1);
      u_sim.reserve(Nsim);
      solve_iters.reserve(Nsim);
      tracking_error.reserve(Nsim);
      x_sim.emplace_back(x_ref[0]);  // push initial state to the front
      for (int i = 0; i < Nsim; ++i) {
        x_sim.emplace_back(Vector::Zero(n));
        u_sim.emplace_back(Vector::Zero(m));
      }

      // Solve
      AltroOptions opts;
      opts.verbose = Verbosity::Silent;
      opts.iterations_max = 80;
      opts.use_backtracking_linesearch = true;
      solver.SetOptions(opts);

      // Initialize variables
      SolveStatus status;
      a_float c_u = 0.5 * u0.dot(Rd.asDiagonal() * u0);
      a_float c;
      Vector q(n);
      Vector u_mpc(m);

      auto t_start = std::chrono::high_resolution_clock::now();
      while (mpc_iter < Nsim) {
        // Solve nonlinear MPC problem
        status = solver.Solve();
        solve_iters.emplace_back(solver.GetIterations());

        // Get control
        solver.GetInput(u_sim[mpc_iter].data(), 0);

        // Simulate the system forward
        dyn(x_sim[mpc_iter + 1].data(), x_sim[mpc_iter].data(), u_sim[mpc_iter].data(), h);

        // Get error from reference
        tracking_error.emplace_back((x_sim[mpc_iter + 1] - x_ref[mpc_iter + 1]).norm());

        // Set new reference trajectory
        ++mpc_iter;
        for (int k = 0; k <= N; ++k) {
          const Vector& xk_ref = x_ref[k + mpc_iter];
          q.noalias() = Qd.asDiagonal() * xk_ref;
          q *= -1;
          c = 0.5 * q.dot(xk_ref);
          c *= -1;
          if (k < N) {
            c += c_u;
          }
          solver.UpdateLinearCosts(q.data(), nullptr, c, k);
        }

        // Set Initial state
        solver.SetInitialState(x_sim[mpc_iter].data(), n);

        // Shift the trajectory
        solver.ShiftTrajectory();
      }
      auto t_end = std::chrono::high_resolution_clock::now();
      using SecondsDouble = std::chrono::duration<double, std::ratio<1>>;
      SecondsDouble t_total = std::chrono::duration_cast<SecondsDouble>(t_end - t_start);
      // fmt::print("Total time = {}\n", t_total);
      // fmt::print("Average rate = {} Hz\n", Nsim / t_total.count());

      // Save trajectory to JSON file
      fs::path test_dir = ALTRO_TEST_DIR;
      fs::path out_file = test_dir / "scotty_mpc.json";
      std::cout<<out_file<<std::endl;
      std::ofstream traj_out(out_file);
      json x_data(x_sim);
      json u_data(u_sim);
      json iters_data(solve_iters);
      json err_data(tracking_error);
      json data;
      data["state_trajectory"] = x_data;
      data["input_trajectory"] = u_data;
      data["N"] = Nsim;
      data["tf"] = Nsim * h;
      data["solve_iters"] = iters_data;
      data["tracking_error"] = err_data;
      traj_out << std::setw(4) << data;
    }
};



int main(int argc, char **argv) {
  altro::ALTROSolver solver(10);
  solver.SetDimension(4, 2, 0, altro::LastIndex);
  fmt::print("Solver Initialized!\n");

  BicycleMPC mpc_solver;
  mpc_solver.SetUp();

  
  mpc_solver.eg1();
  mpc_solver.eg2();
  return 0;
}