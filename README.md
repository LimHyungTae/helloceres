# Hello Ceres! (한글.ver)

---

## C++의 optimization library인 Ceres Solver에 대한 Tutorial

수식적으로 optimization이나 C++ 문법에 대한 상세한 설명보다는 오롯이 **어떻게 쓰는지**에 대한 tutorial



### Tutorial의 목적

Ceres Solver를 설치하면 examples/slam/pose_graph_2d와 examples/slam/pose_graph_3d 경로에 이미 graph SLAM을 위한 error term을 author가 구현해두었습니다. 하지만 막상 저같이 복잡한 C++ 코드에 익숙치 않은 사람이 이해하기가 어려울 수도 있어, step-by-step으로 Ceres Solver를 어떻게 사용하는 지에 대해 배워, 이 라이브러리의 위화감(?)을 없애는 것을 목적으로 합니다

Original author: Hyungtae Lim (shapelim@kaist.ac.kr) <br>
Facilitator : Seungwon Song (sswan55@kaist.ac.kr) 

---

## Ceres 설치

http://ceres-solver.org/installation.html 을 따라 하면 된다


### Setting

<pre><code>$ cmake CMakeLists.txt</code></pre>
<pre><code>$ make</code></pre>

### 코드 구성

- **helloceres_1.cc**: Cost function을 2개 생성해서 addResidualBlock하여 여러 residual cost function에 추가하는 법
- **helloceres_2.cc**: 하나의 cost function 내에서 여러 residual을 선언하는 법
- **helloceres_euclidian_distance.cc**: 하나의 cost function에서 다변수를 입력값으로 주는 법
- **helloceres_pose_2d.cc**: 실제로 2D 상의 pose a와 b에 대한 optimization을 하는 예제
- **helloceres_pose_2d_fixed_a.cc**: helloceres_pose_2d.cc를 실행해보면 예상했던 것과 달리 a와 b 값 둘 다 변경됩니다. 그래서 pose a를 고정시키는 법과, 이걸 통해 SLAM을 하는 데에 있어서 initial Pose의 필요성에 대해 알 수 있습니다.

### 주의

**helloceres_pose_2d.cc**와 **helloceres_pose_2d_fixed_a.cc**에서 원래는 measurements의 불확실성 또한 인자로 고려하기 위해 information matrix가 곱해져야 완전한 error term이 되지만, 여기서는 tool에 대한 설명을 위해 생략되었습니다. (Ceres 기본 라이브러리 examples/slam/pose_graph_2d와 examples/slam/pose_graph_3d에 잘 되어 있어서 그걸 보면 되고, 이 깃허브 페이지는 단순히 tutorial을 위한 자료입니다 :) )
