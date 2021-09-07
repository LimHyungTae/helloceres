#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// 예제 2. 하나의 cost function 내에서 여러 residual을 선언해보기

// 굳이 여러 CostFunctor를 struct로 일일이 만들 필요 없이,
// 하나의 cost function 내에서 여러 residual을 선언하는 것이 가능하다
// p.s. ceres에서 제공하는 대다수의 SLAM 관련 라이브러리들이 하나의 Cost Functor 내에 여러 reisdual을 선언하게 되어있음

struct CostFunctor {
    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = 10.0 - x[0];
        residual[1] = x[0];

        return true;
    }
};

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);

    double       x         = 0.5;
    const double initial_x = x;

    Problem problem;

    // 예제 1과 비교해보면
    // AutoDiffCostFunction<CostFunctor, 2, 1>(new CostFunctor) 에서의
    // {2, 1} <- 이 숫자들의 의미가 각각 Cost function의 {residual의 수, 변수의 수}를 나타내는 것과 같다.
    CostFunction *cost_function =
                         new AutoDiffCostFunction<CostFunctor, 2, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, NULL, &x);

    Solver::Options options;
    options.function_tolerance           = 1e-10;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x
              << " -> " << x << "\n";
    return 0;
}