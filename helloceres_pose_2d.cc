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


// 연습. 두 2D 상의 pose들 optimization해보기
class Pose2dErrorTerm{
  public:
  Pose2dErrorTerm(double measured[])
  :p_measured(measured[0], measured[1]), theta_measured(measured[2]) {}

  template <typename T> bool operator()(const T* const a, const T* const b, T* residual) const {
    // 참고로 값들을 그냥 double로 받으면 타입이 안 맞다고 에러가 뜸!
    // 값들이 ceres::Jet<double, 6>이라는 타입이 되기 때문인 것 같음
    // double x_ab = b[0] - a[0];
    // double y_ab = b[1] - a[1];
    // double theta_ab = b[2] - a[2];
    // 따라서 template을 이용하는 것이 편하다!
    
    // a와 b의 2d position vector 선언
    const Eigen::Matrix<T, 2, 1> p_a(a[0], a[1]);
    const Eigen::Matrix<T, 2, 1> p_b(b[0], b[1]);
    
    // 원점에서 봤을 때 a의 회전 행렬을 구한다.
    // (ceres에도 cos, sin이 있음...math.h 따로 include 안해도 됨!)
    Eigen::Matrix<T, 2, 2> R_a; 
    const T cos = ceres::cos(a[2]);
    const T sin = ceres::sin(a[2]);
    R_a << cos, -sin,
           sin, cos;
    
    //2D pose error terms. 각각 position과 각도에 대한 error를 구함
    const Eigen::Matrix<T, 2, 1> p_diff = R_a.transpose() * (p_b - p_a) - p_measured.cast<T>();
    auto theta_diff = (b[2] - a[2]) - theta_measured;
    
    residual[0] = p_diff(0);
    residual[1] = p_diff(1);
    residual[2] = theta_diff;
    return true;
  }
  private:
  const Eigen::Vector2d p_measured;

  double theta_measured;
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  
  double * a = (double *)malloc(3*sizeof(double));
  a[0] = 3.0;
  a[1] = 1.0;
  a[2] = 0.523599; // radian
  double * b = (double *)malloc(3*sizeof(double));
  b[0] = 5.1;
  b[1] = 2.9;
  b[2] = 0.79; // radian

  //a와 b 사이의 측정된 상대적인 pose
  double * ab_measured = (double *)malloc(3*sizeof(double));
  ab_measured[0] = 2.73205;
  ab_measured[1] = 0.73205;
  ab_measured[2] = 0.2618;

  const std::vector<double> initial_a = {a[0], a[1], a[2]};
  const std::vector<double> initial_b = {b[0], b[1], b[2]};

  // Build the problem.
  Problem problem;

  CostFunction* cost_function =
      new AutoDiffCostFunction<Pose2dErrorTerm, 3, 3, 3>(new Pose2dErrorTerm(ab_measured));
  
  problem.AddResidualBlock(cost_function, NULL, a, b);


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
