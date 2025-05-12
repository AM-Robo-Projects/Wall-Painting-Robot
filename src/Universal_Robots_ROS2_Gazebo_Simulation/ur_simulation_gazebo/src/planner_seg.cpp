#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/filters/extract_indices.h>


int main  ()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_seg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader cloud_reader;
    pcl::PCDWriter cloud_writer;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;


    std::string filename = "/home/abdelrahman/Wall-Painting-Robot/pcd_files/";
    
    cloud_reader.read(filename+std::string("cloud_10.pcd"), *cloud);
    std::cout << "original cloud size: " << cloud->width * cloud->height << std::endl;

    //plane segmentation

    pcl::SACSegmentation<pcl::PointXYZ> plane_seg;
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(0.01); // distance threshold to consider a point as an inlier
    plane_seg.setInputCloud(cloud);
    plane_seg.segment(*inliers, *coefficients);


    // Extract the inliers from the cloud

    pcl::ExtractIndices<pcl::PointXYZ> extract_indicies;

    extract_indicies.setInputCloud(cloud);
    extract_indicies.setIndices(inliers);
    extract_indicies.setNegative(true); // set to true to remove the inliers, false to keep them (extract a plane)
    extract_indicies.filter(*plane_seg_cloud);


    cloud_writer.write <pcl::PointXYZ>(filename+std::string("non_plane_seg.pcd"), *plane_seg_cloud, false);

    
    
    return 0;


}