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

struct CostFunctor {
    template<typename T>
    bool operator()(const T *const a, const T *const b, T *residual) const {
        residual[0] = a[0] - b[0];
        residual[1] = a[1] - b[1];
        residual[2] = a[2] - b[2];
        return true;
    }
};

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);

    // vector a (1.0, 1.0, 1.0) 선언
    // malloc으로 한 것에는 크게 의미가 없습니다.
    // 동적할당 안하고 array로 선언하나, Eigen::Vector3d로 선언하나 상관없음.
    double *a = (double *) malloc(3 * sizeof(double));
    a[0] = 1.0;
    a[1] = 1.0;
    a[2] = 1.0;

    //vector b (1.2, 1.1, 1.01) 선언
    double *b = (double *) malloc(3 * sizeof(double));
    b[0] = 1.2;
    b[1] = 1.1;
    b[2] = 1.01;

    const std::vector<double> initial_a = {a[0], a[1], a[2]};
    const std::vector<double> initial_b = {b[0], b[1], b[2]};


    Problem problem;
    // ############## 매우 중요 ###############
    // line 61에서는 이제 3, 3, 3이 되었는데,
    // 첫 번째 숫자는 optimization의 cost function을 구성하는 residual의 갯수를 의미하고
    // 그 이후의 숫자들은 cost function을 구성하는 변수의 수 N개 만큼 있는데, 각각은 각 변수를 구성하는 변수의 크기 n_i을 적어야 함
    // 이 상황에서는 N: 2 (a, b 두 개이므로 2개의 argument를 더 추가하는데), 각 argument는 a와 b의 변수의 크기 n_i: 3이어서 <> 안에 3, 3, 3이 들어감
    // 즉, {3, 3, 3} <=> {residual 수, a의 변수 수, b의 변수 수}

    CostFunction *cost_function =
                         new AutoDiffCostFunction<CostFunctor, 3, 3, 3>(new CostFunctor);
    // a, b Pointer이기 때문에 예제 1, 2와 달리 a, b 그대로 인풋으로 들어감
    problem.AddResidualBlock(cost_function, NULL, a, b);

    // Run the solver!
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";

    std::cout << "a_x: " << initial_a[0] << " -> " << a[0] << "\n";
    std::cout << "a_y: " << initial_a[1] << " -> " << a[1] << "\n";
    std::cout << "a_z: " << initial_a[2] << " -> " << a[2] << "\n";

    std::cout << "b_x: " << initial_b[0] << " -> " << b[0] << "\n";
    std::cout << "b_y: " << initial_b[1] << " -> " << b[1] << "\n";
    std::cout << "b_z: " << initial_b[2] << " -> " << b[2] << "\n";

    return 0;
}
