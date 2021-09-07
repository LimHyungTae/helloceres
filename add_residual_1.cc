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
    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = 10.0 - x[0];
        return true;
    }
};

// 두 번째 cost function:  0.5 (x)^2
struct CostFunctor2 {
    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = x[0];
        return true;
    }
};


int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);

    // 초기 x 값 선언
    double x = 0.5;

    // 초기의 값과 optimization을 완료한 값을 확인해보기 위해 초기 x 값을 const로 고정시켜 둠
    const double initial_x = x;

    // {1, 1} <- 이 숫자들의 의미가 각각 Cost function의 {residual의 수, 변수의 수}를 나타내는 것과 같다.
    Problem problem;
    // 첫 번째 cost function 선언
    CostFunction *cost_function1 =
                         new AutoDiffCostFunction<CostFunctor1, 1, 1>(new CostFunctor1);
    // 두 번째 cost function 선언
    CostFunction *cost_function2 =
                         new AutoDiffCostFunction<CostFunctor2, 1, 1>(new CostFunctor2);

    problem.AddResidualBlock(cost_function1, NULL, &x);
    problem.AddResidualBlock(cost_function2, NULL, &x);

    // Run the solver!
    Solver::Options options;
    options.eta                          = 1e-4;
    options.function_tolerance           = 1e-10;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x
              << " -> " << x << "\n";
    return 0;
}
