#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>


int main  ()
{
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr voxel_cloud(new pcl::PCLPointCloud2);
    pcl::PCDReader cloud_reader;
    pcl::PCDWriter cloud_writer;


    std::string filename = "/home/abdelrahman/Wall-Painting-Robot/pcd_files/";
    
    cloud_reader.read(filename+std::string("cloud_5.pcd"), *cloud);
    std::cout << "original cloud size: " << cloud->width * cloud->height << std::endl;



    //voxel 

    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;  //decrese the number of points in the cloud while maintaing the shape
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.01f,0.01f,0.01f);  
    voxel_filter.filter(*voxel_cloud);

    std::cout << "voxel cloud size: " << voxel_cloud->width * voxel_cloud->height << std::endl;

    cloud_writer.write(filename+std::string("voxelized.pcd"), *cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true);

    
    
    return 0;


}