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

Eigen::Matrix3d getRz(const double rad) {
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    rot(0, 0) = static_cast<double>(cos(rad));
    rot(0, 1) = static_cast<double>(-sin(rad));
    rot(1, 0) = static_cast<double>(sin(rad));
    rot(1, 1) = static_cast<double>(cos(rad));
    return rot;
}

Eigen::Matrix3d getRy(const double rad) {
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    rot(0, 0) = static_cast<double>(cos(rad));
    rot(0, 2) = static_cast<double>(sin(rad));
    rot(2, 0) = static_cast<double>(-sin(rad));
    rot(2, 2) = static_cast<double>(cos(rad));
    return rot;
}

Eigen::Matrix3d getRx(const double rad) {
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    rot(1, 1) = static_cast<double>(cos(rad));
    rot(1, 2) = static_cast<double>(-sin(rad));
    rot(2, 1) = static_cast<double>(sin(rad));
    rot(2, 2) = static_cast<double>(cos(rad));
    return rot;
}


struct SE3Residual {
    SE3Residual(
            Eigen::Vector3d src,
            Eigen::Vector3d tgt) : src_(src), tgt_(tgt) {}

    template<typename T>
    bool operator()(const T *const rot_est, const T *const ts_est, T *residual) const {
        residual[0] = sqrt(pow(tgt_(0) - (src_(0) - rot_est[2] * src_(1) + rot_est[1] * src_(2) + ts_est[0]), 2)
                           + pow(tgt_(1) - (rot_est[2] * src_(0) + src_(1) - rot_est[0] * src_(2) + ts_est[1]), 2)
                           + pow(tgt_(2) - (rot_est[1] * src_(0) + rot_est[0] * src_(1) + src_(2) + ts_est[2]), 2));

        return true;
    }

private:
    const Eigen::Vector3d src_;
    const Eigen::Vector3d tgt_;
};

using namespace std;

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);


    double lower_bound                               = -20.0;
    double upper_bound                               = 20.0;

    std::random_device                     rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937                           gen(rd());
    std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
    std::default_random_engine             re;


    Eigen::Matrix<double, 4, Eigen::Dynamic> src_h;
    int                                      num_pts = 10;
    src_h.resize(4, num_pts);
    for (int i = 0; i < num_pts; ++i) {

        for (int j  = 0; j < 3; ++j) {
            double random_value = unif(re);
            cout << random_value << endl;
            cout << i << " " << j << endl;
            src_h(j, i) = random_value;
        }
        src_h(3, i) = 1;
    }

    double upper_small_angle_bound = 15.0;
    double upper_translation_bound = 5.0;
    int    num_repeat              = 20;

    for (int l = 0; l < num_repeat; ++l) {
        // Setting GT transformation
        std::uniform_real_distribution<double> unif_rot(-upper_small_angle_bound, upper_small_angle_bound);
        std::uniform_real_distribution<double> unif_ts(-upper_translation_bound, upper_translation_bound);

        const double yaw_gt   = unif_rot(re) * M_PI / 180; // radian
        const double pitch_gt = unif_rot(re) * M_PI / 180; // radian
        const double roll_gt  = unif_rot(re) * M_PI / 180; // radian

        const double x_gt  = unif_ts(re);
        const double y_gt  = unif_ts(re);
        const double z_gt  = unif_ts(re);

        Eigen::Matrix3d rot_gt;
        Eigen::Matrix3d Rz = getRz(yaw_gt);
        Eigen::Matrix3d Ry = getRy(pitch_gt);
        Eigen::Matrix3d Rx = getRx(roll_gt);
        rot_gt = Rz * Ry * Rx;

        Eigen::Matrix4d se3_gt = Eigen::Matrix4d::Identity();
        se3_gt.block<3, 3>(0, 0) = rot_gt;
        se3_gt(0, 3)             = x_gt;
        se3_gt(1, 3)             = y_gt;
        se3_gt(2, 3)             = z_gt;

        Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h = se3_gt * src_h;

        // Homogeneous -> 3 x N
        Eigen::Matrix<double, 3, Eigen::Dynamic> src = src_h.topRows(3);
        Eigen::Matrix<double, 3, Eigen::Dynamic> tgt = tgt_h.topRows(3);

        cout << l << "-th trial" << endl;

        Problem problem;

        double *rot_est = (double *) malloc(3 * sizeof(double));
        rot_est[0] = 0.0;
        rot_est[1] = 0.0;
        rot_est[2] = 0.0;

        double *ts_est = (double *) malloc(3 * sizeof(double));
        ts_est[0] = 0.0;
        ts_est[1] = 0.0;
        ts_est[2] = 0.0;

        for (int i = 0; i < num_pts; ++i) {

            CostFunction *cost_function =
                                 new AutoDiffCostFunction<SE3Residual, 1, 3, 3>(new SE3Residual(src.col(i), tgt.col(i)));
            problem.AddResidualBlock(cost_function, nullptr, rot_est, ts_est);
        }

        // Run the solver!
        Solver::Options options;
        options.minimizer_progress_to_stdout = false;
        Solver::Summary summary;
        Solve(options, &problem, &summary);

        std::cout << summary.BriefReport() << "\n";

        cout<<x_gt<< ", "<< y_gt<< ", " <<z_gt<<endl;
        cout<<ts_est[0]<< ", "<< ts_est[1]<< ", " <<ts_est[2]<<endl;
//        cout << "GT: " << rot_angle_gt * 180 / M_PI << " deg, | Est: " << rot_angle_est * 180 / M_PI << "deg" << endl;
    }

    return 0;

}
