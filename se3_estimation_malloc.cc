// Author: keir@google.com (Keir Mierle)
//
// Tutorial Author: shapelim@kaist.ac.kr (임형태)

#include "ceres/ceres.h"
#include "glog/logging.h"
#include <vector>
#include <random>
#include <iomanip>

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

Eigen::Matrix3d vec2skew_symm(const Eigen::Vector3d &u) {
    Eigen::Matrix3d skew_symm = Eigen::Matrix3d::Zero();
    skew_symm(0, 1) = -u(2);
    skew_symm(1, 0) = u(2);

    skew_symm(0, 2) = u(1);
    skew_symm(2, 0) = -u(1);

    skew_symm(1, 2) = -u(0);
    skew_symm(2, 1) = u(0);

    return skew_symm;
}

Eigen::Matrix3d vec2SO3(double*rot_vec) {
    double mag = 0;

    mag += pow(rot_vec[0], 2);
    mag += pow(rot_vec[1], 2);
    mag += pow(rot_vec[2], 2);
    mag                 = sqrt(mag);

    Eigen::Vector3d u(rot_vec[0] / mag, rot_vec[1] / mag, rot_vec[2] / mag);
    Eigen::Matrix3d R   = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d w_x = vec2skew_symm(u);
    R += sin(mag) * w_x + (1 - cos(mag)) * w_x * w_x;
    return R;
}

Eigen::Matrix4d est2SE3(double*rot_vec, double*ts) {
    Eigen::Matrix3d R = vec2SO3(rot_vec);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;

    T(0, 3) = ts[0];
    T(1, 3) = ts[1];
    T(2, 3) = ts[2];
    return T;
}

double calcTranslationError(Eigen::Matrix4d &rel4x4) {
    double   ts_error = 0;
    for (int i        = 0; i < 3; ++i) {
        ts_error += rel4x4(i, 3) * rel4x4(i, 3);
    }
    ts_error          = sqrt(ts_error);
    return ts_error;
}

// http://www.boris-belousov.net/2016/12/01/quat-dist/
double calcRotationError(Eigen::Matrix4d &rel4x4) {
    Eigen::Matrix3d rel_rot   = rel4x4.block<3, 3>(0, 0);
    double          rot_error = acos((rel_rot.trace() - 1) / 2);
    return rot_error;
}

struct SE3Residual {
    SE3Residual(
            Eigen::Vector3d src,
            Eigen::Vector3d tgt) : src_(src), tgt_(tgt) {}

    template<typename T>
    bool operator()(const T*const rot_est, const T*const ts_est, T*residual) const {
//        residual[0] = sqrt(pow(tgt_(0) - (src_(0) - rot_est[2] * src_(1) + rot_est[1] * src_(2) + ts_est[0]), 2)
//                           + pow(tgt_(1) - (rot_est[2] * src_(0) + src_(1) - rot_est[0] * src_(2) + ts_est[1]), 2)
//                           + pow(tgt_(2) - (rot_est[1] * src_(0) + rot_est[0] * src_(1) + src_(2) + ts_est[2]), 2));
        residual[0] = tgt_(0) - (src_(0) - rot_est[2] * src_(1) + rot_est[1] * src_(2) + ts_est[0]);
        residual[1] = tgt_(1) - (rot_est[2] * src_(0) + src_(1) - rot_est[0] * src_(2) + ts_est[1]);
        residual[2] = tgt_(2) - (-rot_est[1] * src_(0) + rot_est[0] * src_(1) + src_(2) + ts_est[2]);

        return true;
    }

private:
    const Eigen::Vector3d src_;
    const Eigen::Vector3d tgt_;
};

struct SE3ResidualYH {
    SE3ResidualYH(const Eigen::Vector3d &src, const Eigen::Vector3d &tgt) {
        src_x = src[0];
        src_y = src[1];
        src_z = src[2];
        tgt_x = tgt[0];
        tgt_y = tgt[1];
        tgt_z = tgt[2];
    }

    template<typename T>
    bool operator()(const T*const quat_est, const T*const ts_est, T* residual) const {

        Eigen::Map<const Eigen::Quaternion<T>> quat(quat_est);
        Eigen::Matrix<T, 3, 3> rot = quat.toRotationMatrix();

        residual[0] = tgt_x - rot(0,0) * src_x - rot(0,1) * src_y - rot(0,2) * src_z - ts_est[0];
        residual[1] = tgt_y - rot(1,0) * src_x - rot(1,1) * src_y - rot(1,2) * src_z - ts_est[1];
        residual[2] = tgt_z - rot(2,0) * src_x - rot(2,1) * src_y - rot(2,2) * src_z - ts_est[2];

        return true;
    }

private:
    double src_x;
    double src_y;
    double src_z;
    double tgt_x;
    double tgt_y;
    double tgt_z;
};

using namespace std;

int main(int argc, char**argv) {
    google::InitGoogleLogging(argv[0]);


    double lower_bound                               = -20.0;
    double upper_bound                               = 20.0;

    std::random_device                     rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937                           gen(rd());
    std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
    std::default_random_engine             re;

    /*
     * Generate dummy source point cloud
     */
    Eigen::Matrix<double, 4, Eigen::Dynamic> src_h;
    int                                      num_pts = 100;
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

    double upper_small_angle_bound = 10.0;
    double upper_translation_bound = 5.0;
    int    num_repeat              = 20;

    for (int l = 0; l < num_repeat; ++l) {
        // Setting GT transformation
        std::uniform_real_distribution<double> unif_rot(-upper_small_angle_bound, upper_small_angle_bound);
        std::uniform_real_distribution<double> unif_ts(-upper_translation_bound, upper_translation_bound);

        /*
         * Generate Dummy SE(3)
         */
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

        Eigen::Matrix4d T_gt = Eigen::Matrix4d::Identity();
        T_gt.block<3, 3>(0, 0) = rot_gt;
        T_gt(0, 3)             = x_gt;
        T_gt(1, 3)             = y_gt;
        T_gt(2, 3)             = z_gt;

        Eigen::Matrix<double, 4, Eigen::Dynamic> tgt_h = T_gt * src_h;

        // Homogeneous -> 3 x N
        Eigen::Matrix<double, 3, Eigen::Dynamic> src = src_h.topRows(3);
        Eigen::Matrix<double, 3, Eigen::Dynamic> tgt = tgt_h.topRows(3);

        cout << l << "-th trial" << endl;

        Problem problem;

//        double*rot_est = (double*) malloc(3 * sizeof(double));
//        rot_est[0] = 0.0;
//        rot_est[1] = 0.0;
//        rot_est[2] = 0.0;
//
//        double*ts_est = (double*) malloc(3 * sizeof(double));
//        ts_est[0] = 0.0;
//        ts_est[1] = 0.0;
//        ts_est[2] = 0.0;

        Eigen::Quaterniond quat_gt = Eigen::Quaterniond(T_gt.block<3,3>(0, 0));
        Eigen::Vector3d pos_gt = T_gt.block<3,1>(0,3);

        Eigen::Vector3d pos(0.0, 0.0, 0.0);
        Eigen::Quaterniond quat = quat_gt.inverse();

        ceres::EigenQuaternionParameterization *quat_param = new ceres::EigenQuaternionParameterization();

//        problem.AddParameterBlock(pos.data(), 3);
//        problem.AddParameterBlock(quat.coeffs().data(), 4, quat_param);

        for (int i = 0; i < num_pts; ++i) {

//            CostFunction*cost_function =
//                                new AutoDiffCostFunction<SE3Residual, 3, 3, 3>(new SE3Residual(src.col(i), tgt.col(i)));
//            problem.AddResidualBlock(cost_function, nullptr, rot_est, ts_est);

            CostFunction*cost_function =
                                new AutoDiffCostFunction<SE3ResidualYH, 3, 4, 3>(new SE3ResidualYH(src.col(i), tgt.col(i)));
            problem.AddResidualBlock(cost_function, nullptr, quat.coeffs().data(), pos.data());
        }

        // Run the solver!
        Solver::Options options;
        options.minimizer_progress_to_stdout = false;
        Solver::Summary summary;

        Eigen::Quaterniond quat_pre = quat;
        Eigen::Vector3d pos_pre = pos;

        Solve(options, &problem, &summary);

        std::cout << summary.FullReport() << "\n";

        std::cout << std::fixed << std::setprecision(8);
        std::cout << quat_gt.coeffs().transpose() << " | " << pos_gt.transpose() << std::endl;
        std::cout << quat_pre.coeffs().transpose() << " | " << pos_pre.transpose() << std::endl;
        std::cout << quat.coeffs().transpose() << " | " << pos.transpose() << std::endl;

//        Eigen::Matrix4d T_est = est2SE3(rot_est, ts_est);
//        Eigen::Matrix4d T_rel = T_est * T_gt.inverse();
//
//        free(rot_est);
//        free(ts_est);
//
//        cout << "\033[1;32m ABS. size:"<< calcTranslationError(T_gt) << "m / " << calcRotationError(T_gt) * 180.0 / M_PI << "deg -> ";
//        cout << "Rel error: " << calcTranslationError(T_rel) << "m / " << calcRotationError(T_rel) * 180.0 / M_PI << "deg\033[0m" << endl;
    }

    return 0;

}
