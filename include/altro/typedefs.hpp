//
// Created by Brian Jackson on 9/12/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#pragma once

#include <functional>
#include <string>

namespace altro {

using a_float = double;

class ALTROSolver;

constexpr int LastIndex = -1;
constexpr int AllIndices = -2;

enum class SolveStatus {
  Success,
  Unsolved,
  MaxIterations,
  MaxObjectiveExceeded,
  StateOutOfBounds,
  InputOutOfBounds,
  MeritFunGradientTooSmall,
};

// std::string colorToString(SolveStatus status) {
//     switch (status) {
//         case SolveStatus::Success:
//             return "SUCCESS";
//         case SolveStatus::Unsolved:
//             return "UNSOLVED";
//         case SolveStatus::MaxIterations:
//             return "MAX_ITERATIONS";
//         case SolveStatus::MaxObjectiveExceeded:
//             return "MAX_OBJECTIVE_EXCEEDED";
//         case SolveStatus::StateOutOfBounds:
//             return "STATE_OUT_OF_BOUNDS";
//         case SolveStatus::InputOutOfBounds:
//             return "INPUT_OUT_OF_BOUNDS";
//         case SolveStatus::MeritFunGradientTooSmall:
//             return "MERIT_FUN_GRADIENT_TOO_SMALL";
//         default:
//             return "UNKNOWN";
//     }
// }

using CallbackFunction = std::function<void(const ALTROSolver*)>;

using ExplicitDynamicsFunction =
    std::function<void(double* xnext, const double* x, const double* u, float h)>;

using ExplicitDynamicsJacobian =
    std::function<void(double* jac, const double* x, const double* u, float h)>;

using ImplicitDynamicsFunction = std::function<void(double* err, const double* x1, const double* u1,
                                                    const double* x2, const double* u2, float h)>;

using ImplicitDynamicsJacobian =
    std::function<void(double* jac1, double* jac2, const double x1, const double* u1,
                       const double* x2, const double* u2, float h)>;

using CostFunction = std::function<a_float(const a_float* x, const a_float* u)>;

using CostGradient =
    std::function<void(a_float* dx, a_float* du, const a_float* x, const a_float* u)>;

using CostHessian = std::function<void(a_float* ddx, a_float* ddu, a_float* dxdu, const a_float* x,
                                       const a_float* u)>;

using ConstraintFunction = std::function<void(a_float* val, const a_float* x, const a_float* u)>;
using ConstraintJacobian = std::function<void(a_float* jac, const a_float* x, const a_float* u)>;

enum class ConstraintType { EQUALITY, IDENTITY, INEQUALITY, SECOND_ORDER_CONE};

class ConstraintIndex {
 public:
  int KnotPointIndex() const { return k; }

  friend ALTROSolver;

 private:
  ConstraintIndex(int k, int i) : k(k), i(i) {}

  int k;
  int i;
};

}  // namespace altro
