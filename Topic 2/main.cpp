#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char** argv) {
    Eigen::Quaterniond q (0.1,0.35,0.2,0.3);
    Eigen::Matrix3d R;
    R = q.normalized().toRotationMatrix();
    std::cout << "R = \n" << R.matrix() << std::endl;
    std::cout << "R^T = \n" << R.transpose() << std::endl;
    std::cout << "R^-1 = \n" << R.inverse() << std::endl;
    std::cout << "R*R^T = \n" << R * R.transpose() << std::endl;
    return 0;
}
