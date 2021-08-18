/****************************
 * 题目：给定一个稠密的点云，结合前面的练习，对其进行如下操作：
 * 下采样和滤波、重采样平滑、法线计算，贪心投影网格化（请提供结果的截图）。
 *
* 本程序学习目标：
 * 熟悉PCL网格化流程。
 *
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2019.01
****************************/
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{

	// Load input file
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_downSampled(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_smoothed(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile("./data/fusedCloud.pcd", *cloud) == -1)
    {
        cout << "点云数据读取失败！" << endl;
    }

    std::cout << "Orginal points number: " << cloud->points.size() << std::endl;

   	// ----------------------开始你的代码--------------------------//
	// 请参考之前文章中点云下采样，滤波、平滑等内容，以及PCL官网实现以下功能。代码不难。
	
	// 下采样
	pcl::VoxelGrid<PointT> downSampled;
	downSampled.setInputCloud(cloud);
	downSampled.setLeafSize(0.01f, 0.01f, 0.01f);
	downSampled.filter(*cloud_downSampled);
	// 统计滤波
	pcl::StatisticalOutlierRemoval<PointT> statisticalOutlierRemoval;
	statisticalOutlierRemoval.setInputCloud(cloud_downSampled);
	statisticalOutlierRemoval.setMeanK(50);
	statisticalOutlierRemoval.setStddevMulThresh(1.0);
	statisticalOutlierRemoval.filter(*cloud_filtered);
	// 对点云重采样  
    pcl::MovingLeastSquares<PointT, PointT> movingLeastSquares;
    movingLeastSquares.setInputCloud(cloud_filtered);
    movingLeastSquares.setComputeNormals(false);
    movingLeastSquares.setPolynomialOrder(2);
    movingLeastSquares.setPolynomialFit(false);
    pcl::search::KdTree<PointT>::Ptr tree4mls(new pcl::search::KdTree<PointT>);
    movingLeastSquares.setSearchMethod(tree4mls);
    movingLeastSquares.setSearchRadius(0.05);
    movingLeastSquares.process(*cloud_smoothed);
	// 法线估计
	pcl::NormalEstimation<PointT, pcl::PointNormal> normalEstimation;
	normalEstimation.setInputCloud(cloud_smoothed);
	pcl::search::KdTree<PointT>::Ptr tree4n (new pcl::search::KdTree<PointT>);
	normalEstimation.setSearchMethod(tree4n);
	normalEstimation.setRadiusSearch(0.03);
	pcl::PointCloud<pcl::PointNormal>::Ptr normal(new pcl::PointCloud<pcl::PointNormal>);
	normalEstimation.compute(*normal);
	// 将点云位姿、颜色、法线信息连接到一起
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normal (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*normal, *cloud_smoothed, *cloud_with_normal);
//	pcl::io::savePCDFile("./cloud_with_normal.pcd", *cloud_with_normal);
	// 贪心投影三角化
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree4tri (new pcl::search::KdTree<pcl::PointNormal>);
    tree4tri->setInputCloud(cloud_with_normal);
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> greedyProjectionTriangulation;
    greedyProjectionTriangulation.setInputCloud(cloud_with_normal);
    greedyProjectionTriangulation.setSearchRadius(0.1);
    greedyProjectionTriangulation.setMu(2.5);
    greedyProjectionTriangulation.setMaximumNearestNeighbors(50);
    greedyProjectionTriangulation.setMinimumAngle(M_PI / 18);
    greedyProjectionTriangulation.setMaximumAngle(M_PI * 2 / 3);
    greedyProjectionTriangulation.setMaximumSurfaceAngle(M_PI / 4);
    greedyProjectionTriangulation.setNormalConsistency(false);
    greedyProjectionTriangulation.setSearchMethod(tree4tri);
    pcl::PolygonMesh mesh;
    greedyProjectionTriangulation.reconstruct(mesh);
//    cout << "mesh cloud with :  " << mesh.cloud.width;
//    pcl::PointCloud<PointT>::Ptr mesh_cloud(new pcl::PointCloud<PointT>);
//    pcl::fromPCLPointCloud2(mesh.cloud, *mesh_cloud);
//    int index = rand() % 10000;
//    cout << mesh_cloud->points[index].x << " " << mesh_cloud->points[index].y << " "
//          << mesh_cloud->points[index].z ;
//    pcl::io::loadPolygonF
	// ----------------------结束你的代码--------------------------//

//    // 显示网格化结果
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//    viewer->setBackgroundColor(0, 0, 0);  //
//    viewer->addPolygonMesh(mesh, "mesh");  //
//    while (!viewer->wasStopped())
//    {
//    	viewer->spinOnce(100);
//    	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//    }
//    return 1;
//}

