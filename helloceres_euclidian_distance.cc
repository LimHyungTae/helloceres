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
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


// 예제 3. 하나의 cost function에서 multi-variable 인풋으로 받기

// 실제 복잡한 문제는 다변수로 구성되어 있기 때문에, 다변수를 인풋을 받는 법에 대해 이해해야 한다.
// 하나의 cost functor에서 다변수 optimization해보기
// vector a (1.0, 1.0, 1.0)과 vector b (1.2, 1.1, 1.01)의 euclidian distance 최소화하기
// p.s. 엄밀히 말하면 Manhattan distance를 최소화하는 것이라고 볼수 있지만,
//      SLAM에서 position optimization할 때 저렇게 각각 x, y, z에 대해 minimization을 함. 


struct CostFunctor {
  template <typename T> bool operator()(const T* const a, const T* const b, T* residual) const {
    residual[0] = a[0] - b[0];
    residual[1] = a[1] - b[1];
    residual[2] = a[2] - b[2];
    return true;
  }
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  
  // vector a (1.0, 1.0, 1.0) 선언 
  // malloc으로 한 것에는 크게 의미가 없습니다. 
  // 동적할당 안하고 array로 선언하나, Eigen::Vector3d로 선언하나 상관없음.
  // Eigen 으로 선언하면 53-55번째 줄 element에 접근하는 모양만 a(0) - b(0) 같은 식으로 달라질 뿐
  double * a = (double *)malloc(3*sizeof(double));
  a[0] = 1.0;
  a[1] = 1.0;
  a[2] = 1.0;

  //vector b (1.2, 1.1, 1.01) 선언
  double * b = (double *)malloc(3*sizeof(double));
  b[0] = 1.2;
  b[1] = 1.1;
  b[2] = 1.01;

  const std::vector<double> initial_a = {a[0], a[1], a[2]};
  const std::vector<double> initial_b = {b[0], b[1], b[2]};


  Problem problem;
  // ############## 매우 중요 ###############
  // line 96에서는 이제 3, 3, 3이 되었는데, 
  // 첫 번째 숫자는 optimization의 cost function을 구성하는 residual의 갯수를 의미하고
  // 그 이후의 숫자들은 cost function을 구성하는 변수의 수 N개 만큼 있는데, 각각은 각 변수를 구성하는 변수의 크기 n을 적어야 함
  // 이 상황에서는 N: 2 (a, b 두 개이므로 2개의 argument를 더 추가하는데), 각 argument는 a와 b의 변수의 크기 n: 3이어서 <> 안에 3, 3, 3이 들어감

  CostFunction* cost_function =
      new AutoDiffCostFunction<CostFunctor, 3, 3, 3>(new CostFunctor);
  // a, b Pointer이기 때문에 예제 1, 2와 달리 a, b 그대로 인풋으로 들어감
  problem.AddResidualBlock(cost_function, NULL, a, b);

  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << "\n";
  
  std::cout << "a_x: " <<initial_a[0] << " -> " << a[0] << "\n";
  std::cout << "a_y: " <<initial_a[1] << " -> " << a[1] << "\n";
  std::cout << "a_z: " <<initial_a[2] << " -> " << a[2] << "\n";

  std::cout << "b_x: " <<initial_b[0] << " -> " << b[0] << "\n";
  std::cout << "b_y: " <<initial_b[1] << " -> " << b[1] << "\n";
  std::cout << "b_z: " <<initial_b[2] << " -> " << b[2] << "\n";
  
  return 0;
}
