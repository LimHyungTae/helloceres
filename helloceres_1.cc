// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// A simple example of using the Ceres minimizer.
//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)

#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// 예제 1. Cost function을 2개 생성해서 addResidualBlock 해보기

// 첫 번째 cost function:  0.5 (10 - x)^2 
struct CostFunctor1 {
  template <typename T> bool operator()(const T* const x, T* residual) const {
    residual[0] = 10.0 - x[0];
    return true;
  }
};
// 두 번째 cost function:  0.5 (x)^2 
struct CostFunctor2 {
  template <typename T> bool operator()(const T* const x, T* residual) const {
    residual[0] = x[0];
    return true;
  }
};


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  // 초기 x 값 선언
  double x = 0.5;

  // 초기의 값과 optimization을 완료한 값을 확인해보기 위해 초기 x 값을 const로 고정시켜 둠
  const double initial_x = x;


  Problem problem;
  // 첫 번째 cost function 선언
  CostFunction* cost_function1 =
      new AutoDiffCostFunction<CostFunctor1, 1, 1>(new CostFunctor1);
  // 두 번째 cost function 선언
  CostFunction* cost_function2 =
      new AutoDiffCostFunction<CostFunctor2, 1, 1>(new CostFunctor2);
  
  problem.AddResidualBlock(cost_function1, NULL, &x);
  problem.AddResidualBlock(cost_function2, NULL, &x);

  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x
            << " -> " << x << "\n";
  return 0;
}
