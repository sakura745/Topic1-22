/****************************
 * 题目：给定一个轨迹１，数据格式：timestamp tx ty tz qx qy qz qw
 * 自定义一个任意的旋转矩阵和平移向量（可以尝试不同的值看看结果有什么变化），对轨迹１进行变换，得到一个新的轨迹２
 * 使用ICP算法（提示：取平移作为三维空间点）估计轨迹１，２之间的位姿，然后将该位姿作用在轨迹２
 * 验证：ICP算法估计的旋转矩阵和平移向量是否准确；轨迹１，２是否重合
 * 
* 本程序学习目标：
 * 熟悉ICP算法的原理及应用。
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM，参考答案请到星球内查看。
 * 时间：2019.05
 * 作者：小六
****************************/
#include <iostream>
#include "sophus/se3.hpp"
#include <fstream>
#include <pangolin/pangolin.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include "unistd.h"

using namespace std;
using namespace cv;

void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> pose1,
                    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> pose2);



int main()
{
    // path to trajectory file
    string trajectory_file = "./trajectory.txt";


    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> pose_groundtruth;
    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> pose_new;
    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> pose_estimate;
    vector<Point3f> pts_new, pts_groundtruth;

    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    Sophus::SE3d T;
    string timestamp;
    ifstream textFile;

    // 自定义一个变换矩阵
    /**********************　开始你的代码，参考星球里作业５代码　****************************/
    // 旋转向量（轴角）：沿Z轴旋转45°
    Eigen::AngleAxisd rotation_vector( M_PI / 4, Eigen::Vector3d(0,0,1));
    // 平移向量，可以自己自定义，我这里是 x=3, y=-1, z=0.8，可以多尝试其他值
    Eigen::Vector3d tranlation (3, -1, 0.8);
    /**********************　结束你的代码　****************************/

    Sophus::SE3d myTransform(rotation_vector.matrix(), tranlation);
    cout<<"rotation matrix =\n"<< rotation_vector.matrix() <<endl;
    cout<<"translation vector =\n"<<tranlation <<endl;
    cout<<"myTransform =\n"<<myTransform.matrix() <<endl;

    textFile.open(trajectory_file);

    //　读取轨迹数据
    /**********************　开始你的代码，参考星球里作业８代码　****************************/
	// 提示：取平移作为三维空间点
    string readLine;
    if(!textFile.is_open()){
        cerr << "Oops! Cannt find the file in " << trajectory_file << endl;
        return -1;
    }
    else {
        while (getline(textFile, readLine) && !readLine.empty()) {
            istringstream readWord(readLine);
            readWord >> timestamp >> t[0] >> t[1] >> t[2] >> q.x() >> q.y() >> q.z() >> q.w();
            if ('#' == readLine[0]) continue;

            T = Sophus::SE3d (q,t);
            pose_groundtruth.push_back(T);
            pts_groundtruth.push_back(Point3f (t[0], t[1], t[2]));

            Sophus::SE3d Tmp(myTransform * T);
            pose_new.push_back(Tmp);
            pts_new.push_back(Point3f (Tmp.translation()[0], Tmp.translation()[1], Tmp.translation()[2]));

        }
    }
    /**********************　结束你的代码　****************************/
    textFile.close();

	
    // 使用ICP算法估计轨迹１，２之间的位姿，然后将该位姿作用在轨迹２
    /**********************　开始你的代码，参考十四讲中第７章ICP代码　****************************/
    Eigen::Vector3d P_center, P_prime_center;

    for(int i = 0; i < pts_groundtruth.size(); i++){
        Eigen::Vector3d P (pts_new[i].x, pts_new[i].y, pts_new[i].z);
        Eigen::Vector3d P_prime (pts_groundtruth[i].x, pts_groundtruth[i].y, pts_groundtruth[i].z);

        P_center += P;
        P_prime_center += P_prime;
    }

    //质心
    P_center = P_center / (int)pts_groundtruth.size();
    P_prime_center = P_prime_center / (int)pts_groundtruth.size();


    Eigen::Matrix3d W = Eigen::Matrix3d::Zero(), R;
    Eigen::Vector3d tt;
    for(int i = 0; i < pose_groundtruth.size(); i++){
        W += Eigen::Vector3d (pts_new[i].x - P_center.x(), pts_new[i].y - P_center.y(),
                                pts_new[i].z - P_center.z())
             * Eigen::Vector3d (pts_groundtruth[i].x - P_prime_center.x(), pts_groundtruth[i].y - P_prime_center.y(),
                               pts_groundtruth[i].z - P_prime_center.z()).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU|Eigen::ComputeFullV);
    R = svd.matrixU() * svd.matrixV().transpose();
    if(R.determinant() < 0) R = -R;

    tt = P_center - R * P_prime_center;

    cout << "Estimated result from ICP:\n";
    cout << "Estimated rotation matrix = \n" << R << endl;
    cout << "Estimated translation vector = \n" << tt << endl;

    /**********************　结束你的代码　****************************/

//    DrawTrajectory(pose_groundtruth, pose_new);  // 变换前的两个轨迹
    DrawTrajectory(pose_groundtruth, pose_new);  // 轨迹应该是重合的
    return 0;
}



void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> pose1,
                    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> pose2) {
    if (pose1.empty()||pose2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < pose1.size() - 1; i++) {
            glColor3f(1 - (float) i / pose1.size(), 0.0f, (float) i / pose1.size());
            glBegin(GL_LINES);
            auto p1 = pose1[i], p2 = pose1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < pose2.size() - 1; i++) {
            glColor3f(1 - (float) i / pose2.size(), 0.0f, (float) i / pose2.size());
            glBegin(GL_LINES);
            auto p1 = pose2[i], p2 = pose2[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}
