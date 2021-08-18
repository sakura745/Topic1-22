/****************************
 * 题目：已知旋转矩阵定义是沿着Z轴旋转45°。
 * 按照该定义初始化旋转向量、旋转矩阵、四元数、欧拉角。请编程实现：
 * 1、以上四种表达方式的相互转换关系并输出，看看是否正确
 * 2、假设平移向量为（1,2,3）,请输出旋转矩阵和该平移矩阵构成的欧式变换矩阵，并根据欧式变换矩阵提取旋转向量及平移向量
 *
* 本程序学习目标：
 * 学习eigen中刚体旋转的四种表达方式，熟悉他们之间的相互转换关系
 * 熟悉旋转平移和欧式变换矩阵的相互转换关系
 *
 * 作者：公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2018.09
****************************/
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;
using namespace Eigen;

int main ( int argc, char** argv )
{
    Eigen::Matrix3d rotation_matrix;
    Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d ( 0,0,1 ) );
    rotation_matrix = rotation_vector;
    Eigen::Vector3d euler_angles1, euler_angles2;

    cout << "Rotation_Matrix = \n" << rotation_matrix << endl
         << "Rotation_Vector angle = \n" << rotation_vector.angle() << endl
         << "Rotation_Vector axis = \n" << rotation_vector.axis().transpose() << endl;

    euler_angles1 = rotation_matrix.eulerAngles(2,1,0);
    euler_angles2 = rotation_matrix.eulerAngles(0,1,2);
    cout << "Euler Angle = \n" << euler_angles1.transpose() << endl
         << "Euler Angle = \n" << euler_angles2.transpose();
    return 0;
}
