//
// Created by shapelim on 21. 9. 8..
//

#include "ceres/ceres.h"
#include "glog/logging.h"

using namespace std;

void foo(Eigen::Vector3d *b) {
    cout << (*b)(0) << ", " << (*b)(1) << ", " << (*b)(2) << endl;
}

int main() {
    Eigen::Vector3d a(0.0, 1.1, 1.3);
    foo(&a);
    return 0;
}