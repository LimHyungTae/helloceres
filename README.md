<div align="center">

<h1>Hello Ceres!</h1>

<img src="https://img.shields.io/badge/Ubuntu%2022.04-passing-brightgreen?logo=ubuntu" alt="Ubuntu 22.04"/>
<img src="https://img.shields.io/badge/Ubuntu%2024.04-passing-brightgreen?logo=ubuntu" alt="Ubuntu 24.04"/>
<img src="https://img.shields.io/badge/macOS%2015-passing-brightgreen?logo=apple" alt="macOS 15"/>
<img src="https://img.shields.io/badge/Ceres-2.x-blue" alt="Ceres 2.x"/>
<img src="https://img.shields.io/badge/C%2B%2B-17-blue?logo=cplusplus" alt="C++17"/>

<p><strong><em>A step-by-step Ceres Solver tutorial for Graph SLAM.</em></strong></p>

<p>🇰🇷 <a href="README_ko.md">한국어 README</a></p>

</div>

Original author: Hyungtae Lim (shapelim@kaist.ac.kr)

______________________________________________________________________

## :rocket: Overview

This is a tutorial for Ceres Solver, a C++ optimization library.

Rather than diving into the mathematical details of optimization or C++ syntax, this tutorial focuses purely on **how to use** Ceres Solver in practice.

It is intended for those who are not yet comfortable with the optimization part of Graph SLAM. In theory, studying Graph SLAM can be overwhelming because of the heavy equations, but in practice Ceres handles the non-linear optimization for us, and the error terms are already provided in `examples/slam/pose_graph_2d` and `examples/slam/pose_graph_3d`, so all we need to do is use them. :)

This repository is meant to help those who want to write Graph SLAM from the low level but find C++ code intimidating. It walks through how to use Ceres Solver step by step.

______________________________________________________________________

## :books: Table of Contents

* **EX 1**: [helloworld.cc](https://github.com/LimHyungTae/helloceres/blob/master/helloworld.cc): The simplest example
* **EX 2-1**: [add_residual_1.cc](https://github.com/LimHyungTae/helloceres/blob/master/add_residual_1.cc): Adding an extra CostFunctor
* **EX 2-2**: [add_residual_2.cc](https://github.com/LimHyungTae/helloceres/blob/master/add_residual_2.cc): Combining multiple CostFunctors into a single cost function
* **EX 3**: [multivariate.cc](https://github.com/LimHyungTae/helloceres/blob/master/multivariate.cc): Taking multiple variables as input
* **EX 4-1**: [helloceres_pose_2d.cc](https://github.com/LimHyungTae/helloceres/blob/master/helloceres_pose_2d.cc): Optimizing two 2D poses
* **EX 4-2**: [helloceres_pose_2d_fixed_a.cc](https://github.com/LimHyungTae/helloceres/blob/HEAD/helloceres_pose_2d_fixed_a.cc): Fixing variables (uncommon approach)
* **EX 4-3**: [helloceres_pose_2d_fixed_b.cc](https://github.com/LimHyungTae/helloceres/blob/HEAD/helloceres_pose_2d_fixed_a.cc): Fixing variables (common approach)
* **EX-5**: ...

______________________________________________________________________

## :hammer: Build

### Local (macOS / Linux)

```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

> **macOS note (Anaconda users):** If you have Anaconda installed alongside Homebrew, pin the Homebrew dependencies explicitly to avoid a gflags/glog conflict:
> ```bash
> cmake -B build -S . -DCMAKE_BUILD_TYPE=Release \
>       -DCMAKE_BUILD_RPATH=/opt/homebrew/lib \
>       -DCMAKE_IGNORE_PATH=/opt/anaconda3/lib \
>       -Dgflags_DIR=/opt/homebrew/lib/cmake/gflags \
>       -Dglog_DIR=/opt/homebrew/lib/cmake/glog
> ```

### Docker (Ubuntu 22.04 or 24.04)

```bash
# Ubuntu 22.04
docker build --build-arg UBUNTU_VER=22.04 -t helloceres:ubuntu22 .
docker run --rm helloceres:ubuntu22

# Ubuntu 24.04
docker build --build-arg UBUNTU_VER=24.04 -t helloceres:ubuntu24 .
docker run --rm helloceres:ubuntu24
```

______________________________________________________________________

## :test_tube: Test

```bash
bash scripts/run_tests.sh build   # or build_mac on macOS
```

______________________________________________________________________

## :warning: Note

![img](img/rot_no_convergence.png)
