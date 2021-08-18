/****************************
 * 题目：现有一个运动着的相机拍摄的连续两张图片，其中特征点匹配部分已经完成。
 * 请根据两帧图像对应的匹配点计算基础矩阵，并利用该矩阵绘制出前10个特征点对应的极线。
 *
* 本程序学习目标：
 * 理解掌握对极约束的原理
 * 熟悉OpenCV编程
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2018.11
****************************/
#include<iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;
int main( int argc, char** argv )
{

    Mat rgb1 = imread( "../rgb1.ppm");
    Mat rgb2 = imread( "../rgb2.ppm");

    Ptr<FeatureDetector> detector;
    Ptr<DescriptorExtractor> descriptor;
    
//    detector = FeatureDetector::create("ORB");
//    descriptor = DescriptorExtractor::create("ORB");
//
    detector = ORB::create();
    descriptor = ORB::create();

    vector< KeyPoint > kp1, kp2;
    detector->detect( rgb1, kp1 );
    detector->detect( rgb2, kp2 );

    // 计算描述子
    Mat desp1, desp2;
    descriptor->compute( rgb1, kp1, desp1 );
    descriptor->compute( rgb2, kp2, desp2 );

    // 匹配描述子
    vector< DMatch > matches;
    BFMatcher matcher;
    matcher.match( desp1, desp2, matches );
    cout<<"Find total "<<matches.size()<<" matches."<<endl;

    // 筛选匹配对
    vector< DMatch > goodMatches;
    double minDis = 9999;
    for ( size_t i=0; i<matches.size(); i++ )
    {
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }

    for ( size_t i=0; i<matches.size(); i++ )
    {
        if (matches[i].distance < 10*minDis)
            goodMatches.push_back( matches[i] );
    }


    vector< Point2f > pts1, pts2;
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        pts1.push_back(kp1[goodMatches[i].queryIdx].pt);
        pts2.push_back(kp2[goodMatches[i].trainIdx].pt);
    }

    // 请先计算基础矩阵并据此绘制出前10个匹配点对应的对极线，可以调用opencv函数
    // ----------- 开始你的代码 --------------//
    Mat F_matrix = findFundamentalMat(pts1, pts2, CV_FM_RANSAC );
    vector<Vec3f> lines1, lines2;
    computeCorrespondEpilines(pts1, 1, F_matrix, lines1);
    computeCorrespondEpilines(pts2, 2, F_matrix, lines2); 
    RNG rng;
    for ( int index = 0; index < 10; index++ ) {
        Scalar color = Scalar(rng(256),rng(256),rng(256));
        circle(rgb1, pts1[index], 4, color, 2);
        circle(rgb2, pts2[index], 4, color, 2);
        line(rgb1, Point2f(0, -lines1[index][2]/lines1[index][1]),
             Point2f(rgb1.cols, -(lines1[index][0] * rgb1.cols + lines1[index][2]) / lines1[index][1]),
             color, 1 );
        line(rgb2, Point2f(0, -lines2[index][2]/lines2[index][1]),
             Point2f(rgb2.cols, -(lines2[index][0] * rgb2.cols + lines2[index][2]) / lines2[index][1]),
             color, 1 );
    }
    // ----------- 结束你的代码 --------------//
    imshow("epiline1", rgb2);
    imshow("epiline2", rgb1);
    waitKey(0);
    return 0;
}
