/****************************
 * 题目：四元数球面线性插值
 * 我们用智能手机采集了图像序列和IMU数据，由于IMU帧率远大于图像帧率，需要你用Slerp方法进行四元数插值，使得插值后的IMU和图像帧对齐
 * 已知某帧图像的时间戳为：t =700901880170406，离该图像帧最近的前后两个时刻IMU时间戳为：
 * t1 = 700901879318945，t2 = 700901884127851
 * IMU在t1, t2时刻测量得的旋转四元数为：
 * q1x=0.509339, q1y=0.019188, q1z=0.049596, q1w=0.858921；
 * q2x=0.509443, q2y=0.018806, q2z=0.048944,q2w=0.858905
 * 根据上述信息求IMU对齐到图像帧的插值后的四元数
* 本程序学习目标：
 * 学习四元数球面线性插值方法，注意实际使用中需要考虑的问题。
 *
 * 作者：公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2019.08
****************************/
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
using namespace Eigen;

// 四元数球面线性插值简化方法：v'=v1*cosθ' + v⊥*sinθ'，原理见公众号推送文章
 Quaterniond slerp(double t, Quaterniond &q1, Quaterniond &q2)
 {
    // 归一化四元数
    q1.normalize();
    q2.normalize();

    // 四元数变为Mat: 方法 1
//    double q1_array[4] = { q1.w(), q1.x(), q1.y(), q1.z()};
//    double q2_array[4] = { q2.w(), q2.x(), q2.y(), q2.z()};
//    Mat q1_Mat(4,1,CV_64F, q1_array);
//    Mat q2_Mat(4, 1, CV_64F, q2_array);

     // 四元数变为Mat: 方法 2
    Mat q1_Mat = (Mat_<double>(4, 1) << q1.w(), q1.x(), q1.y(), q1.z());
    Mat q2_Mat = (Mat_<double>(4, 1) << q2.w(), q2.x(), q2.y(), q2.z());

    double dotProd = q1_Mat.dot(q2_Mat);
    double norm2 = norm(q1_Mat, NORM_L2) * norm(q2_Mat, NORM_L2);
    double cosTheta = dotProd / norm2;

    Mat result;
    Quaterniond result_quat;
    if (cosTheta > 0.9995f){
        result = (1.0f - t)*q1_Mat + t*q2_Mat;
    }
    else
	{
        double theta = acosf(cosTheta);
        double thetaT = theta * t;
        Mat qperp = q2_Mat - cosTheta*q1_Mat;
        result = q1_Mat * cosf(thetaT) + qperp * sinf(thetaT);
    }

    result = result / norm(result, NORM_L2);
	// Mat 转化为四元数
    result_quat.w() = result.at<double>(0, 0);
    result_quat.x() = result.at<double>(1, 0);
    result_quat.y() = result.at<double>(2, 0);
    result_quat.z() = result.at<double>(3, 0);
    return result_quat;
 }
int main ( int argc, char** argv )
{
	Quaterniond q1 = Quaterniond(0.858921, 0.509339, 0.019188, 0.049596);
	Quaterniond q2 = Quaterniond(0.858905, 0.509443, 0.018806, 0.048944);
	double t_img(700901880170406), t1_imu(700901879318945), t2_imu(700901884127851);
	
	double t = (t_img - t1_imu) / (t2_imu - t1_imu);
    Quaterniond q_slerp = slerp(t, q1, q2);
	cout<<"插值后的四元数：q_slerp =\n"<< q_slerp.coeffs() <<endl;  //coeffs的顺序是(x,y,z,w)
	
    return 0;
}
