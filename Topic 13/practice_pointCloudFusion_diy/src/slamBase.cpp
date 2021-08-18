#include "slamBase.hpp"

PointCloud::Ptr image2PointCloud(Mat rgb, Mat depth, CAMERA_INTRINSIC_PARAMETERS camera)
{
    PointCloud::Ptr cloud (new PointCloud());
    for (size_t m = 0; m < depth.rows; m++)
        for (size_t n = 0; n < depth.cols; n++) {
            ushort d = depth.ptr<ushort>(m)[n];
            if ( 0 == d) continue;

            PointT p;
            p.z = double (d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;

            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

            cloud->points.push_back(p);
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    return cloud;
}

PointCloud::Ptr tranPointCloud(PointCloud::Ptr &src_cloud, Eigen::Isometry3d T){
    PointCloud::Ptr result(new PointCloud);

    for(auto &i : src_cloud->points){
        Eigen::Vector3d tmp_vector(i.x, i.y, i.z);
        Eigen::Vector3d result_vector = T * tmp_vector;
        PointT p;
        p.x = result_vector.x();
        p.y = result_vector.y();
        p.z = result_vector.z();
        p.b = i.b;
        p.g = i.g;
        p.r = i.r;
        result->points.push_back(p);

    }
    cout << "Changed done!" << endl;
    return result;
};


PointCloud::Ptr pointCloudFusion( PointCloud::Ptr &original, FRAME& newFrame,
                                  Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS camera )
{

	PointCloud::Ptr newCloud (new PointCloud()), tranCloud(new PointCloud());
	newCloud = image2PointCloud(newFrame.rgb, newFrame.depth, camera);
//	pcl::transformPointCloud(*newCloud, *tranCloud, T.matrix());
    tranCloud = tranPointCloud(newCloud, T);
	*original += *tranCloud;
	return original;
}


void readCameraTrajectory(string camTransFile, vector<Eigen::Isometry3d> &poses)
{
    ifstream fcamTrans(camTransFile);
    if(!fcamTrans.is_open())
    {
        cerr << "trajectory is empty!" << endl;
        return;
    }

    string readLine;
    while ( getline(fcamTrans, readLine) && !readLine.empty() ) {
        istringstream readWord(readLine);
        Eigen::Vector3d t;
        Eigen::Quaterniond q;
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        if ('#' == readLine[0] ) continue;
        readWord >> t[0] >> t[1] >> t[2] >> q.x() >> q.y() >> q.z() >> q.w();
        T.rotate(q);
        T.pretranslate(t);
        poses.push_back(T);

    }
   	// ---------- 开始你的代码  ------------- -//
	// 参考作业8 绘制轨迹
	// ---------- 结束你的代码  ------------- -//
}