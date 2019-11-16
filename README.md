# Hello Ceres! (한글.ver)

---

## C++의 optimization library인 Ceres Solver에 대한 Tutorial

수식적으로 optimization이나 C++ 문법에 대한 상세한 설명보다는 오롯이 **어떻게 쓰는지**에 대한 tutorial

Original author: Hyungtae Lim (shapelim@kaist.ac.kr) <br>
Facilitator : Seungwon Song (sswan55@kaist.ac.kr) 

## Ceres 설치

http://ceres-solver.org/installation.html 을 따라 하면 된다


### Easy-implemented Monte Carlo Localization (MCL) code on ros-kinetic

These codes are implemented only using OpenCV library! So It might be helpful for newbies to understand overall MCL procedures

Originally, it is RE510 materials at KAIST implemented by Seungwon Song as a TA.

```cpp
int main() {
  int y = SOME_MACRO_REFERENCE;
  int x = 5 + 6;
  cout << "Hello World! " << x << std::endl();
}
```

### Dependency libraries

* Eigen (default version of ROS)
* opencv (default version of ROS)
---

## Results



<pre><code>$ roscore</code></pre>

0. Setting
    1. Download this repository 
    <pre><code>$ cd /home/$usr_name/catkin_ws/src</code></pre>
    <pre><code>$ https://github.com/LimHyungTae/mcl_2d_lidar_ros.git</code></pre>
    2. Build this ros code as follows.
    <pre><code>$ cd /home/$usr_name/catkin_ws</code></pre>
    <pre><code>$ catkin_make re510_slam</code></pre>

    Or if you use catkin-tools, then type below line on the command
    <pre><code>$ catkin build re510_slam</code></pre>

1. Mapgen

    1. Move to the repository e.g,
  
    <pre><code>$ cd /home/$usr_name/catkin_ws/src/mcl_2d_lidar_ros</code></pre>

    2. Play rosbag **re510_mapgen.bag**

    <pre><code>$ rosbag play rosbag/re510_mapgen.bag -r 3</code></pre>

    3. Run mapgen code

    <pre><code>$ rosrun re510_slam rs_mapgen</code></pre>

2. MCL

    1. Move to the repository e.g,
  
    <pre><code>$ cd /home/$usr_name/catkin_ws/src/mcl_2d_lidar_ros</code></pre>

    2. Play rosbag **re510_mcl.bag**

    <pre><code>$ rosbag play rosbag/re510_mcl.bag</code></pre>

    3. Change the paths of png: 7 and 8th lines on the re510_slam/rs_mcl/src/mcl.cpp
    
    4. Run MCL code

    <pre><code>$ rosrun re510_slam rs_mcl</code></pre>


## Consideration

**/vrpn_client_node/turtleBot/pose**: Pose captured from OptiTrack, which is a motion caputre system(Ground Truth).

**/odom**: 2D pose from Turtlebot2.

**/scan**: 2D LiDAR data measured by RP LiDAR A1M8.

