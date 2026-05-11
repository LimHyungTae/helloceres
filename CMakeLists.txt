cmake_minimum_required(VERSION 3.14)
project(hello_ceres)
set(CMAKE_CXX_STANDARD 17)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# Only Ceres itself should be compiled with CERES_BUILDING_SHARED_LIBRARY
if(BUILD_SHARED_LIBS)
  remove_definitions(-DCERES_BUILDING_SHARED_LIBRARY)
endif()

add_executable(helloworld helloworld.cc)
target_link_libraries(helloworld Ceres::ceres Eigen3::Eigen)

add_executable(add_residual_1 add_residual_1.cc)
target_link_libraries(add_residual_1 Ceres::ceres Eigen3::Eigen)

add_executable(add_residual_2 add_residual_2.cc)
target_link_libraries(add_residual_2 Ceres::ceres Eigen3::Eigen)

add_executable(multivariate multivariate.cc)
target_link_libraries(multivariate Ceres::ceres Eigen3::Eigen)

add_executable(pitch_estimation small_pitch_estimation.cc)
target_link_libraries(pitch_estimation Ceres::ceres Eigen3::Eigen)

add_executable(se3_estimation se3_estimation.cc)
target_link_libraries(se3_estimation Ceres::ceres Eigen3::Eigen)

add_executable(se3_estimation_malloc se3_estimation_malloc.cc)
target_link_libraries(se3_estimation_malloc Ceres::ceres Eigen3::Eigen)

add_executable(pass_by_address auxiliary/pass_by_address.cpp)
target_link_libraries(pass_by_address Ceres::ceres Eigen3::Eigen)

#add_executable(helloceres_pose2d helloceres_pose_2d.cc)
#target_link_libraries(helloceres_pose2d Ceres::ceres Eigen3::Eigen)
#
#add_executable(helloceres_pose2d_fixed_a helloceres_pose_2d_fixed_a.cc)
#target_link_libraries(helloceres_pose2d_fixed_a Ceres::ceres Eigen3::Eigen)
#
#add_executable(helloceres_pose2d_fixed_b helloceres_pose_2d_fixed_b.cc)
#target_link_libraries(helloceres_pose2d_fixed_b Ceres::ceres Eigen3::Eigen)
