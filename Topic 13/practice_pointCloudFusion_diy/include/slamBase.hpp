# pragma once //保证头文件只被编译一次

#include <iostream>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// camera instrinsic parameters
struct CAMERA_INTRINSIC_PARAMETERS
{
    double fx, fy, cx, cy, scale;
};

struct FRAME
{
    cv::Mat rgb, depth;
};

PointCloud::Ptr image2PointCloud(Mat rgb, Mat depth, CAMERA_INTRINSIC_PARAMETERS camera);
PointCloud::Ptr pointCloudFusion( PointCloud::Ptr &original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS camera );
PointCloud::Ptr tranPointCloud(PointCloud::Ptr &src_cloud, Eigen::Isometry3d T);
void readCameraTrajectory(string camTransFile, vector<Eigen::Isometry3d> &poses);
