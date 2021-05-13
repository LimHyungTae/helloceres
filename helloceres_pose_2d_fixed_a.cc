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
// Tutorial Author: shapelim@kaist.ac.kr (임형태)

#include "ceres/ceres.h"
#include "glog/logging.h"
#include <vector>
#include <Eigen/Dense>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class Pose2dErrorTerm{
  public:
  Pose2dErrorTerm(double measured[])
  :p_measured(measured[0], measured[1]), theta_measured(measured[2]) {}

  template <typename T> bool operator()(const T* const a, const T* const b, T* residual) const {
    const Eigen::Matrix<T, 2, 1> p_a(a[0], a[1]);
    const Eigen::Matrix<T, 2, 1> p_b(b[0], b[1]);
    
    Eigen::Matrix<T, 2, 2> R_a;
    const T cos = ceres::cos(a[2]);
    const T sin = ceres::sin(a[2]);
    R_a << cos, -sin,
           sin, cos;
    
    const Eigen::Matrix<T, 2, 1> p_diff = R_a.transpose() * (p_b - p_a) - p_measured.cast<T>();

    auto theta_diff = (b[2] - a[2]) - theta_measured;
    
    residual[0] = p_diff(0);
    residual[1] = p_diff(1);
    residual[2] = theta_diff;
    // ##############중요##############
    // ceres_pose_2d에서 optimization을 하면 a와 b가 둘 다 움직여버림.
    // 따라서 a를 고정시키기 위해 a에 대한 residual을 cost function에 추가함으로써 
    // a가 최대한 고정되도록할 수 있음. 
    // 비고: 이 방식은 잘 쓰이지 않습니다. 다음 예제에서 더 흔히 쓰는 방식을 소개함.
    residual[3] = 3.000001 - a[0];
    residual[4] = 1.000001 - a[1];
    residual[5] = 0.523599000001 - a[2];
    return true;
  }
  private:
  const Eigen::Vector2d p_measured;

  double theta_measured;
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  
  double * a = (double *)malloc(3*sizeof(double));
  a[0] = 3.0;
  a[1] = 1.0;
  a[2] = 0.523599; // radian
  double * b = (double *)malloc(3*sizeof(double));
  b[0] = 5.1;
  b[1] = 2.9;
  b[2] = 0.79;

  double * ab_measured = (double *)malloc(3*sizeof(double));
  ab_measured[0] = 2.73205;
  ab_measured[1] = 0.73205;
  ab_measured[2] = 0.2618;
  
  const std::vector<double> initial_a = {a[0], a[1], a[2]};
  const std::vector<double> initial_b = {b[0], b[1], b[2]};

  
  Problem problem;

  CostFunction* cost_function =
      new AutoDiffCostFunction<Pose2dErrorTerm, 6, 3, 3>(new Pose2dErrorTerm(ab_measured));
  
  problem.AddResidualBlock(cost_function, NULL, a, b);

  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  
  std::cout << "a(x, y, theta): (" <<initial_a[0]<<", "<<initial_a[1]<<", "<< initial_a[2] << ")\n";
  std::cout << "--> " <<a[0]<<", "<<a[1]<<", "<< a[2] << ")\n";
  
  std::cout << "b(x, y, theta): (" <<initial_b[0]<<", "<<initial_b[1]<<", "<< initial_b[2] << ")\n";
  std::cout << "--> " <<b[0]<<", "<<b[1]<<", "<< b[2] << ")\n";

  return 0;
}
