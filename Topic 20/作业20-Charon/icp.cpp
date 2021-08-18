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
#include "sophus/se3.h"
#include <fstream>
#include <pangolin/pangolin.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

using namespace std;
using namespace cv;

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose1,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose2);



int main()
{
    // path to trajectory file
    string trajectory_file = "./trajectory.txt";


    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose_groundtruth;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose_new;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose_estimate;
    vector<Point3f> pts_new, pts_groundtruth;

    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    Sophus::SE3 T;
    string timestamp;
    ifstream textFile;

    // 自定义一个变换矩阵
    /**********************　开始你的代码，参考星球里作业５代码　****************************/
    // 旋转向量（轴角）：沿Z轴旋转45°
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d rotate = rotation_vector.matrix();                      // 旋转矩阵
    // 平移向量，可以自己自定义，我这里是 x=3, y=-1, z=0.8，可以多尝试其他值
    Eigen::Vector3d tranlation(3, -1, 0.8);

    /**********************　结束你的代码　****************************/
    Sophus::SE3 myTransform(rotate,tranlation);
    cout<<"rotation matrix =\n"<<rotation_vector.matrix() <<endl;
    cout<<"translation vector =\n"<<tranlation <<endl;
    cout<<"myTransform =\n"<<myTransform.matrix() <<endl;

    textFile.open(trajectory_file.c_str());

    //　读取轨迹数据
    /**********************　开始你的代码，参考星球里作业８代码　****************************/
	// 提示：取平移作为三维空间点
    if(!textFile.is_open())
    {
        cout << "Failed to open txtx file ..." << endl;
        return 1;
    }
    double times, x, y, z, qx, qy, qz, qw;
    string line;
    while(getline(textFile, line))
    {
        if(line.empty() || line[0] == '#')
            continue;
        stringstream record(line);
        record >> times >> x >> y >> z >> qx >> qy >> qz >> qw;
        Eigen::Vector3d t(x, y, z);
        Eigen::Quaterniond q(qw, qx, qy, qz);
        Sophus::SE3 T(q, t);

        // 1. 加入相机在参考帧的位姿
        pose_groundtruth.push_back(T);

        // 2. 加入相机在参考帧的空间位置
        pts_groundtruth.push_back(cv::Point3f(x, y, z));

        // 3. 计算相机在当前帧的位姿
        Sophus::SE3 T_ = myTransform.inverse() * T;
        pose_new.push_back(T_);

        // 4. 计算相机在当前帧的空间位置
        Eigen::Vector3d t1(x, y, z);
        Eigen::Vector3d t2 = myTransform.inverse()*t1;
        pts_new.push_back(cv::Point3f(t2[0], t2[1], t2[2]));
    }
    /**********************　结束你的代码　****************************/
    textFile.close();

    // 使用ICP算法估计轨迹１，２之间的位姿，然后将该位姿作用在轨迹２
    /**********************　开始你的代码，参考十四讲中第７章ICP代码　****************************/

    // 1. 计算质心坐标
    Point3f p1, p2;     // center of mass
    int N = pts_groundtruth.size();
    for(int i = 0; i < N; i++)
    {
        p1 += pts_groundtruth[i];
        p2 += pts_new[i];
    }
    p1 = Point3f(Vec3f(p1) / N);
    p2 = Point3f(Vec3f(p2) / N);


    // 2. 计算去质心坐标
    vector<Point3f> q1(N);
    vector<Point3f> q2(N);
    for(int i = 0; i < N; i++)
    {
        q1[i] = pts_groundtruth[i] - p1;
        q2[i] = pts_new[i] - p2;
    }


    // 3. 计算矩阵W  q1 * q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for(int i = 0; i < N; i++)
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    //cout << "W = \n" << W << endl;


    // 4. SVD 分解 W , 计算R
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    if(U.determinant() * V.determinant() < 0)
    {
        for(int x = 0; x < 3; x++)
            U(x, 2) *= -1;
    }
    //cout << "U=\n"<<U<<endl;
    //cout << "V=\n"<<V<<endl;
    Eigen::Matrix3d R_ = U*(V.transpose());
    cout << "R extimate = \n" << R_ << endl;

    // 5. 由旋转矩阵R, 计算平移     t_ = p - Rp
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_*Eigen::Vector3d(p2.x, p2.y, p2.z);
    cout << "t extimate = \n" << t_ << endl;

    // 6. 将轨迹2变换T_
    Sophus::SE3 T_(R_, t_);
    for(int i = 0; i < N; i++)
        pose_estimate.push_back(T_ * pose_new[i]);

    /**********************　结束你的代码　****************************/

    //DrawTrajectory(pose_groundtruth, pose_new);  // 变换前的两个轨迹
    DrawTrajectory(pose_groundtruth, pose_estimate);  // 轨迹应该是重合的
    return 0;
}



void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose1,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> pose2) {
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
