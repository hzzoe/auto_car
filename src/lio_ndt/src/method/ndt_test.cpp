#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    // 加载点云数据 
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/room_scan1.pcd", *target_cloud) == -1)
    {
        PCL_ERROR("Could not find pcd \n");
        return (-1);
    }
    std::cout << "load " << target_cloud->size() << " data points from target_cloud" << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/room_scan2.pcd", *source_cloud) == -1)
    {
        PCL_ERROR("Could not find pcd \n");
        return (-1);
    }
    std::cout << "load " << source_cloud->size() << " data points from source_cloud" << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);   // 滤波之后的点数据
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;                      // 滤波器

    approximate_voxel_filter.setLeafSize(0.3, 0.3, 0.3);
    approximate_voxel_filter.setInputCloud(source_cloud);
    approximate_voxel_filter.filter(*filter_cloud);                                         // source_cloud -> filter_cloud

    std::cout << "Filter cloud contain " << filter_cloud->size() << "data points from source_cloud" << endl;

    Eigen::AngleAxisf init_rotation(0.69, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translasition(1.0, 0, 0);
    Eigen::Matrix4f init_guss = (init_translasition * init_rotation).matrix();

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);  // 输出点云


    // =================================================   ndt   =========================================================
    // 补全相关代码

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    //设置参数
    ndt.setInputTarget(target_cloud);
    ndt.setInputSource(filter_cloud);
    ndt.setTransformationEpsilon(1e-5);  // 变换矩阵的最小变化量阈值
    ndt.setStepSize(0.1);                // 步长
    ndt.setResolution(1.0);              // 搜索半径 分辨率
    ndt.setMaximumIterations(35);        // 最大迭代次数
    
    ndt.align(*target_cloud, init_guss);

    pcl::transformPointCloud(*filter_cloud, *output_cloud, init_guss);  //1->2 by3
    
    //获取得分
    if (ndt.hasConverged())
    {
        std::cout << "Normal Distributions Transform has converged" << std::endl;
        std::cout << "score : " << ndt.getFitnessScore() << std::endl;
    }
    else
    {
        std::cout << "Not converged." << std::endl;
    }
    
    
    // ===================================================   ndt   ==========================================================
    



    // =============================================  visualization  ========================================================

    pcl::visualization::PCLVisualizer::Ptr viewer_final(new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    // 目标点云 红色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target_cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");


    // 转换后的云 绿色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output_cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output_cloud");
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    // 等待直到可视化窗口关闭
    while (!viewer_final->wasStopped())
    {
        viewer_final->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
    return 0;
}