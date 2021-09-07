// Author: keir@google.com (Keir Mierle)
//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)

#include "ceres/ceres.h"
#include "glog/logging.h"
#include <vector>
#include <random>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

Eigen::Matrix3d GetRotMat(const double pitch_angle_rad) {
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    rot(0, 0) = static_cast<double>(cos(pitch_angle_rad));
    rot(0, 2) = static_cast<double>(sin(pitch_angle_rad));
    rot(2, 0) = static_cast<double>(-sin(pitch_angle_rad));
    rot(2, 2) = static_cast<double>(cos(pitch_angle_rad));
    return rot;
}

struct RotResidual {
    RotResidual(
            Eigen::Vector3d src,
            Eigen::Vector3d tgt) : src_(src), tgt_(tgt) {}

    template<typename T>
    bool operator()(const T *const angle_est, T *residual) const {
        // Use small angle assumption
        /*
         *  |q_0|    | cos(angle) 0 sin(angle)|   |p_0|
         *  |q_1|  = |     0      1     0     | * |p_1|
         *  |q_2|    |-sin(angle) 0 cos(angle)|   |p_2|
         */
        residual[0] = sqrt(pow(tgt_(0) - src_(0) - src_(2) * (*angle_est), 2)
                           + pow(tgt_(1) - src_(1), 2)
                           + pow(tgt_(2) - src_(2) + src_(0) * (*angle_est), 2));
        return true;
    }

private:
    const Eigen::Vector3d src_;
    const Eigen::Vector3d tgt_;
};

using namespace std;

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);


    Eigen::Matrix<double, 3, Eigen::Dynamic> src;
    Eigen::Matrix<double, 3, Eigen::Dynamic> tgt;

    double lower_bound = -20.0;
    double upper_bound = 20.0;

    std::random_device                     rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937                           gen(rd());
    std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
    std::default_random_engine             re;

    int num_pts = 10;
    src.resize(3, num_pts);
    for (int i = 0; i < num_pts; ++i) {

        for (int j = 0; j < 3; ++j) {
            double random_value = unif(re);
            cout << random_value << endl;
            cout << i << " " << j << endl;
            src(j, i) = random_value;
        }
    }

    double upper_small_angle_bound = 15.0;
    int    num_repeat              = 20;

    for (int l = 0; l < num_repeat; ++l) {
        std::uniform_real_distribution<double> unif2(0, upper_small_angle_bound);
        const double                           rot_angle_gt = unif2(re) * M_PI / 180; // radian

        Eigen::Matrix3d rot_gt = GetRotMat(rot_angle_gt);
        cout << l << "-th trial" << endl;
        tgt                    = rot_gt * src;

        Problem  problem;
        double   rot_angle_est = 0;
        for (int i             = 0; i < num_pts; ++i) {

            CostFunction *cost_function =
                                 new AutoDiffCostFunction<RotResidual, 1, 1>(new RotResidual(src.col(i), tgt.col(i)));
            problem.AddResidualBlock(cost_function, nullptr, &rot_angle_est);
        }

        // Run the solver!
        Solver::Options options;
        options.minimizer_progress_to_stdout = false;
        Solver::Summary summary;
        Solve(options, &problem, &summary);

        std::cout << summary.BriefReport() << "\n";
        cout << "GT: " << rot_angle_gt * 180 / M_PI << " deg, | Est: " << rot_angle_est * 180 / M_PI << "deg" << endl;
    }

    return 0;

}
