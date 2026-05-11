# helloceres Polish & Ceres 2.x Migration — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Migrate helloceres to Ceres 2.x API, modernize CMake, validate on Ubuntu 22.04 / 24.04 (Docker) and macOS 15 (local), then polish README.md (EN) and add README_ko.md (KO).

**Architecture:** Fix one dead Ceres 1.x API call in `se3_estimation_malloc.cc`, modernize `CMakeLists.txt` to use imported targets (`Ceres::ceres`, `Eigen3::Eigen`), add a single `Dockerfile` with an `UBUNTU_VER` build-arg, and a `scripts/run_tests.sh` that runs all eight executables. README polish and badges come last, after all three platforms confirm green.

**Tech Stack:** C++17, Ceres 2.x, Eigen3, CMake 3.14+, Docker, Bash, shields.io static badges

---

## File Map

| Action | Path | Purpose |
|--------|------|---------|
| Modify | `CMakeLists.txt` | Use `Ceres::ceres` + `Eigen3::Eigen` targets, remove hardcoded paths |
| Modify | `se3_estimation_malloc.cc` | Remove `EigenQuaternionParameterization` dead code, update comments |
| Modify | `helloworld.cc` | `NULL` → `nullptr` |
| Modify | `add_residual_1.cc` | `NULL` → `nullptr` |
| Modify | `add_residual_2.cc` | `NULL` → `nullptr` |
| Modify | `multivariate.cc` | `NULL` → `nullptr` |
| Create | `scripts/run_tests.sh` | Run all executables and report PASS/FAIL |
| Create | `Dockerfile` | Ubuntu 22.04 / 24.04 shared build image |
| Modify | `README.md` | format-readme structure + platform badges |
| Create | `README_ko.md` | Korean version of README.md |

---

## Task 1: Fix Ceres 2.x API — remove dead EigenQuaternionParameterization code

**Files:**
- Modify: `se3_estimation_malloc.cc:243` (and surrounding dead-code block)

`ceres::EigenQuaternionParameterization` was removed in Ceres 2.x. Line 243 declares `quat_param` but its usage is entirely commented out — it is pure dead code that blocks compilation.

- [ ] **Step 1.1: Remove the dead declaration and update the Manifold comment**

In `se3_estimation_malloc.cc`, replace the block starting at line 237 (after `Eigen::Vector3d pos_gt = ...`):

```cpp
        Eigen::Quaterniond pos_gt = T_gt.block<3,1>(0,3);  // already there

        Eigen::Vector3d pos(0.0, 0.0, 0.0);
        Eigen::Quaterniond quat = quat_gt.inverse();

        ceres::EigenQuaternionParameterization *quat_param = new ceres::EigenQuaternionParameterization();

//        problem.AddParameterBlock(pos.data(), 3);
//        problem.AddParameterBlock(quat.coeffs().data(), 4, quat_param);
```

Replace with (remove `quat_param`, update comment to show Ceres 2.x Manifold API):

```cpp
        Eigen::Vector3d pos(0.0, 0.0, 0.0);
        Eigen::Quaterniond quat = quat_gt.inverse();

        // Ceres 2.x Manifold API (replaces deprecated LocalParameterization):
        //   ceres::EigenQuaternionManifold quat_manifold;
        //   problem.SetManifold(quat.coeffs().data(), &quat_manifold);
```

- [ ] **Step 1.2: Commit**

```bash
git add se3_estimation_malloc.cc
git commit -m "fix: remove Ceres 1.x EigenQuaternionParameterization dead code"
```

---

## Task 2: Replace NULL with nullptr across source files

**Files:**
- Modify: `helloworld.cc:42`
- Modify: `add_residual_1.cc:52,53`
- Modify: `add_residual_2.cc:39`
- Modify: `multivariate.cc:63`

- [ ] **Step 2.1: helloworld.cc — replace NULL**

Change:
```cpp
    problem.AddResidualBlock(cost_function, NULL, &x);
```
To:
```cpp
    problem.AddResidualBlock(cost_function, nullptr, &x);
```

- [ ] **Step 2.2: add_residual_1.cc — replace NULL (two occurrences)**

Change:
```cpp
    problem.AddResidualBlock(cost_function1, NULL, &x);
    problem.AddResidualBlock(cost_function2, NULL, &x);
```
To:
```cpp
    problem.AddResidualBlock(cost_function1, nullptr, &x);
    problem.AddResidualBlock(cost_function2, nullptr, &x);
```

- [ ] **Step 2.3: add_residual_2.cc — replace NULL**

Change:
```cpp
    problem.AddResidualBlock(cost_function, NULL, &x);
```
To:
```cpp
    problem.AddResidualBlock(cost_function, nullptr, &x);
```

- [ ] **Step 2.4: multivariate.cc — replace NULL**

Change:
```cpp
    problem.AddResidualBlock(cost_function, NULL, a, b);
```
To:
```cpp
    problem.AddResidualBlock(cost_function, nullptr, a, b);
```

- [ ] **Step 2.5: Commit**

```bash
git add helloworld.cc add_residual_1.cc add_residual_2.cc multivariate.cc
git commit -m "style: replace NULL with nullptr in AddResidualBlock calls"
```

---

## Task 3: Modernize CMakeLists.txt

**Files:**
- Modify: `CMakeLists.txt`

Replace the entire file with:

```cmake
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
```

- [ ] **Step 3.1: Write the new CMakeLists.txt** (content above)

- [ ] **Step 3.2: Commit**

```bash
git add CMakeLists.txt
git commit -m "build: modernize CMakeLists.txt with Ceres::ceres and Eigen3::Eigen targets"
```

---

## Task 4: Create scripts/run_tests.sh

**Files:**
- Create: `scripts/run_tests.sh`

- [ ] **Step 4.1: Create scripts/ directory and run_tests.sh**

```bash
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${1:-${SCRIPT_DIR}/../build}"

BINARIES=(
    helloworld
    add_residual_1
    add_residual_2
    multivariate
    pitch_estimation
    se3_estimation
    se3_estimation_malloc
    pass_by_address
)

PASS=0
FAIL=0

for bin in "${BINARIES[@]}"; do
    exe="${BUILD_DIR}/${bin}"
    if [ ! -f "$exe" ]; then
        echo "MISSING: $exe"
        ((FAIL++))
        continue
    fi
    echo "--- Running ${bin} ---"
    if "${exe}" > /dev/null 2>&1; then
        echo "PASS: ${bin}"
        (( PASS += 1 ))
    else
        echo "FAIL: ${bin} (exit code $?)"
        (( FAIL += 1 ))
    fi
done

echo ""
echo "==============================="
echo "Results: ${PASS} passed, ${FAIL} failed"
echo "==============================="
[ "${FAIL}" -eq 0 ]
```

- [ ] **Step 4.2: Make executable and commit**

```bash
chmod +x scripts/run_tests.sh
git add scripts/run_tests.sh
git commit -m "test: add run_tests.sh to verify all executables pass"
```

---

## Task 5: Create Dockerfile

**Files:**
- Create: `Dockerfile`

- [ ] **Step 5.1: Write Dockerfile**

```dockerfile
ARG UBUNTU_VER=22.04
FROM ubuntu:${UBUNTU_VER}

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    cmake \
    build-essential \
    libceres-dev \
    libgoogle-glog-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . .

RUN cmake -B build -S . -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j"$(nproc)"

CMD ["bash", "scripts/run_tests.sh", "/workspace/build"]
```

- [ ] **Step 5.2: Commit**

```bash
git add Dockerfile
git commit -m "ci: add Dockerfile for Ubuntu 22.04/24.04 build and test"
```

---

## Task 6: Test on macOS 15 (local)

**Files:** none — local validation only

- [ ] **Step 6.1: Install Ceres via Homebrew**

```bash
brew install ceres-solver
```

Expected: ceres-solver 2.2.0 installed

- [ ] **Step 6.2: Configure and build out-of-source**

```bash
cmake -B build_mac -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build_mac -j$(sysctl -n hw.logicalcpu)
```

Expected: all 8 executables built with no errors or warnings.

- [ ] **Step 6.3: Run all tests**

```bash
bash scripts/run_tests.sh build_mac
```

Expected output ends with:
```
Results: 8 passed, 0 failed
```

- [ ] **Step 6.4: Record result** — macOS 15 = PASS (used in Task 8 badge)

---

## Task 7: Test on Ubuntu 22.04 (Docker)

**Files:** none — Docker validation only

- [ ] **Step 7.1: Build Docker image for Ubuntu 22.04**

```bash
docker build --build-arg UBUNTU_VER=22.04 -t helloceres:ubuntu22 .
```

Expected: image builds successfully, all 8 targets compile.

- [ ] **Step 7.2: Run tests in container**

```bash
docker run --rm helloceres:ubuntu22
```

Expected output ends with:
```
Results: 8 passed, 0 failed
```

- [ ] **Step 7.3: Record result** — Ubuntu 22.04 = PASS (used in Task 8 badge)

---

## Task 8: Test on Ubuntu 24.04 (Docker)

**Files:** none — Docker validation only

- [ ] **Step 8.1: Build Docker image for Ubuntu 24.04**

```bash
docker build --build-arg UBUNTU_VER=24.04 -t helloceres:ubuntu24 .
```

Expected: image builds successfully.

- [ ] **Step 8.2: Run tests in container**

```bash
docker run --rm helloceres:ubuntu24
```

Expected output ends with:
```
Results: 8 passed, 0 failed
```

- [ ] **Step 8.3: Record result** — Ubuntu 24.04 = PASS (used in Task 9 badge)

---

## Task 9: Polish README.md — format-readme + badges

**Files:**
- Modify: `README.md`

Prerequisites: Tasks 6, 7, 8 all PASS.

- [ ] **Step 9.1: Add platform badges to README.md**

Add the following badge block inside the `<div align="center">` header section.
Only add badges for platforms that actually passed in Tasks 6–8.

Ubuntu 22.04 badge (add if Task 7 PASS):
```html
<img src="https://img.shields.io/badge/Ubuntu%2022.04-passing-brightgreen?logo=ubuntu" alt="Ubuntu 22.04"/>
```

Ubuntu 24.04 badge (add if Task 8 PASS):
```html
<img src="https://img.shields.io/badge/Ubuntu%2024.04-passing-brightgreen?logo=ubuntu" alt="Ubuntu 24.04"/>
```

macOS 15 badge (add if Task 6 PASS):
```html
<img src="https://img.shields.io/badge/macOS%2015-passing-brightgreen?logo=apple" alt="macOS 15"/>
```

Ceres version badge (always):
```html
<img src="https://img.shields.io/badge/Ceres-2.x-blue" alt="Ceres 2.x"/>
```

C++ standard badge (always):
```html
<img src="https://img.shields.io/badge/C%2B%2B-17-blue?logo=cplusplus" alt="C++17"/>
```

- [ ] **Step 9.2: Apply format-readme skill**

Invoke the `format-readme` skill to reformat the full README.md per its structure rules (centered header, section dividers, emoji headings). Existing content is preserved verbatim; only structure changes.

- [ ] **Step 9.3: Add Docker build section to README.md**

Add a **Build** section with:
```markdown
## :whale: Build

### Local (macOS / Linux)
```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

### Docker (Ubuntu 22.04 or 24.04)
```bash
# Ubuntu 22.04
docker build --build-arg UBUNTU_VER=22.04 -t helloceres:ubuntu22 .
docker run --rm helloceres:ubuntu22

# Ubuntu 24.04
docker build --build-arg UBUNTU_VER=24.04 -t helloceres:ubuntu24 .
docker run --rm helloceres:ubuntu24
```
```

- [ ] **Step 9.4: Commit**

```bash
git add README.md
git commit -m "docs: reformat README.md with format-readme structure and platform badges"
```

---

## Task 10: Create README_ko.md

**Files:**
- Create: `README_ko.md`

- [ ] **Step 10.1: Write README_ko.md**

Korean version mirroring README.md structure. Content based on existing Korean text in the original README.md, expanded to cover all sections. Use the same format-readme structure (centered header, section dividers, emoji headings). Include same badges as README.md (platform results already known).

Sections:
1. Centered header (title, badges — same as README.md)
2. `:rocket: 개요` — tutorial purpose (Korean text from original README)
3. `:books: 예제 목록` — Table of Contents (Korean labels)
4. `:whale: 빌드` — local + Docker build instructions in Korean
5. `:test_tube: 테스트` — run_tests.sh usage in Korean
6. `:warning: 주의사항` — the rotation convergence warning + image

- [ ] **Step 10.2: Commit**

```bash
git add README_ko.md
git commit -m "docs: add Korean README (README_ko.md)"
```
