cmake_minimum_required(VERSION 3.10)
project(hello_ceres)

include_directories("/usr/include/eigen3")
find_package(Ceres REQUIRED)
include_directories(
 # include
 ${CERES_INCLUDE_DIRS}
)

# Should be larger than version 14 to avoid ‘integer_sequence’ is not a member of ‘std’ error
set(CMAKE_CXX_STANDARD 17)

# Only Ceres itself should be compiled with CERES_BUILDING_SHARED_LIBRARY
# defined, any users of Ceres will have CERES_USING_SHARED_LIBRARY defined
# for them in Ceres' config.h if appropriate.
if (BUILD_SHARED_LIBS)
  remove_definitions(-DCERES_BUILDING_SHARED_LIBRARY)
endif()

add_executable(helloworld helloworld.cc)
target_link_libraries(helloworld ceres)

#################
add_executable(add_residual_1 add_residual_1.cc)
target_link_libraries(add_residual_1 ceres)

add_executable(add_residual_2 add_residual_2.cc)
target_link_libraries(add_residual_2 ceres)

add_executable(multivariate multivariate.cc)
target_link_libraries(multivariate ceres)

add_executable(pitch_estimation small_pitch_estimation.cc)
target_link_libraries(pitch_estimation ceres)

add_executable(se3_estimation se3_estimation.cc)
target_link_libraries(se3_estimation ceres)

add_executable(se3_estimation_malloc se3_estimation_malloc.cc)
target_link_libraries(se3_estimation_malloc ceres)
#
#add_executable(helloceres_pose2d helloceres_pose_2d.cc)
#target_link_libraries(helloceres_pose2d ceres)
#
#add_executable(helloceres_pose2d_fixed_a helloceres_pose_2d_fixed_a.cc)
#target_link_libraries(helloceres_pose2d_fixed_a ceres)
#
#add_executable(helloceres_pose2d_fixed_b helloceres_pose_2d_fixed_b.cc)
#target_link_libraries(helloceres_pose2d_fixed_b ceres)
#################
add_executable(pass_by_address auxiliary/pass_by_address.cpp)
target_link_libraries(pass_by_address ceres)
