cmake_minimum_required(VERSION 3.10)
project(hello_ceres)
# Ceres Solver - A fast non-linear least squares minimizer
# Copyright 2015 Google Inc. All rights reserved.
# http://ceres-solver.org/
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Google Inc. nor the names of its contributors may be
#   used to endorse or promote products derived from this software without
#   specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: keir@google.com (Keir Mierle)

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
add_executable(helloceres_1 helloceres_1.cc)
target_link_libraries(helloceres_1 ceres)

add_executable(helloceres_2 helloceres_2.cc)
target_link_libraries(helloceres_2 ceres)

add_executable(helloceres_ed helloceres_euclidian_distance.cc)
target_link_libraries(helloceres_ed ceres)

add_executable(helloceres_rotation helloceres_small_angle_rotation.cc)
target_link_libraries(helloceres_rotation ceres)

add_executable(helloceres_pose2d helloceres_pose_2d.cc)
target_link_libraries(helloceres_pose2d ceres)

add_executable(helloceres_pose2d_fixed_a helloceres_pose_2d_fixed_a.cc)
target_link_libraries(helloceres_pose2d_fixed_a ceres)

add_executable(helloceres_pose2d_fixed_b helloceres_pose_2d_fixed_b.cc)
target_link_libraries(helloceres_pose2d_fixed_b ceres)
#################


