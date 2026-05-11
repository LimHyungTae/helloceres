# helloceres Polish & Ceres 2.x Migration — Design Spec

**Date:** 2026-05-12
**Repo:** helloceres (Ceres Solver tutorial, Korean)
**Goal:** Migrate to Ceres 2.x API, modernize CMake, add Docker-based CI test, polish README (EN + KO).

______________________________________________________________________

## 1. Ceres 2.x API Migration

### Problem
`se3_estimation_malloc.cc:243` instantiates `ceres::EigenQuaternionParameterization`, which was removed in Ceres 2.x. Its usage is entirely commented out — it is dead code that breaks compilation.

### Change
- Delete the dead declaration (`quat_param`) and the associated commented-out `AddParameterBlock` calls in `se3_estimation_malloc.cc`.
- Replace `NULL` with `nullptr` in `AddResidualBlock` calls across all source files:
  - `helloworld.cc`
  - `add_residual_1.cc`
  - `add_residual_2.cc`
  - `multivariate.cc`

### Files touched
`se3_estimation_malloc.cc`, `helloworld.cc`, `add_residual_1.cc`, `add_residual_2.cc`, `multivariate.cc`

______________________________________________________________________

## 2. CMakeLists.txt Modernization

### Changes
- Bump `cmake_minimum_required` to 3.14.
- Remove `include_directories("/usr/include/eigen3")` (hardcoded, non-portable).
- Add `find_package(Eigen3 REQUIRED NO_MODULE)` to locate Eigen properly.
- Replace bare `ceres` link target with `Ceres::ceres` (imported target from Ceres 2.x config).
- Add `Eigen3::Eigen` to each `target_link_libraries` call (carries include dirs automatically).
- Keep commented-out pose2d targets as-is (intentionally disabled for tutorial scope).

### Rationale
`Ceres::ceres` is the modern CMake imported target provided by Ceres 2.x's `CeresConfig.cmake`. It carries transitive include dirs and compile flags, making manual `include_directories` calls unnecessary. Same pattern for `Eigen3::Eigen`.

______________________________________________________________________

## 3. Docker + Test Script

### Dockerfile
- Single `Dockerfile` with `ARG UBUNTU_VER=22.04` at the top.
- Supports `22.04` (Ceres 2.0) and `24.04` (Ceres 2.2) via `--build-arg`.
- Installs: `cmake build-essential libceres-dev libgoogle-glog-dev libeigen3-dev`.
- Out-of-source build: copies repo into `/workspace`, builds in `/workspace/build`.
- Default `CMD` runs `scripts/run_tests.sh`.

### scripts/run_tests.sh
Shell script that:
1. Iterates over all built executables: `helloworld`, `add_residual_1`, `add_residual_2`, `multivariate`, `pitch_estimation`, `se3_estimation`, `se3_estimation_malloc`, `pass_by_address`.
2. Runs each, checks exit code.
3. Prints PASS/FAIL per binary; exits non-zero if any fail.

### Test matrix
| Platform | Method | Ceres version |
|---|---|---|
| Ubuntu 22.04 | Docker `--build-arg UBUNTU_VER=22.04` | 2.0.x |
| Ubuntu 24.04 | Docker `--build-arg UBUNTU_VER=24.04` | 2.2.x |
| macOS 15 | Local cmake (Homebrew Ceres) | system |

______________________________________________________________________

## 4. README Polish

### README.md (English)
- Reformat using `format-readme` skill structure: centered header, section dividers, emoji headings.
- Add **Build** section: local cmake instructions + Docker instructions.
- Add **Test** section: `run_tests.sh` usage.
- Preserve all existing content verbatim.

### README_ko.md (Korean)
- New file based on existing Korean content in README.md.
- Same structure as README.md.
- Includes Korean explanations of each example.
- Build/test sections mirrored in Korean.

______________________________________________________________________

## 5. Out-of-Scope

- No changes to algorithm logic, numerical behavior, or residual implementations.
- No reorganization of source files into subdirectories.
- `helloceres_pose_2d*.cc` targets remain commented out in CMakeLists.txt.
