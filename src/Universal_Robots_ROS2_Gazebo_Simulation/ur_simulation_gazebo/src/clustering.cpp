#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>



typedef pcl::PointXYZ PointT;

void cloud_saver (const std::string& filename, std::string& path,pcl::PointCloud<PointT>::Ptr cloud_arg)
{
    
    pcl::PCDWriter cloud_writer;
    cloud_writer.write<pcl::PointXYZ>(path+std::string(filename), *cloud_arg, false);


}

void find_wall_boundries (pcl::PointCloud<PointT>::Ptr extracted_walls){


// Compute the bounding box{
PointT min_pt, max_pt;

pcl::getMinMax3D(*extracted_walls, min_pt, max_pt);
std::cout << "Bounding box min (bottom left corner): " << min_pt << std::endl; 
std::cout << "Bounding box max (top right corner): " << max_pt << std::endl;

float width = std::abs(max_pt.x - min_pt.x);
float height = std::abs(max_pt.z - min_pt.z);
float depth = std::abs(max_pt.y - min_pt.y);

std::vector<PointT> wall_corners(4);


std::cout << "\nBottom corners:" << std::endl;
wall_corners[0] = PointT(min_pt.x, max_pt.y, min_pt.z);  // bottom-left-front
wall_corners[1] = PointT(min_pt.x, max_pt.y, max_pt.z);  // top-left-front

std::cout << wall_corners[0] << std::endl;
std::cout << wall_corners[1] << std::endl;  

std::cout << "\nTop corners:" << std::endl;

wall_corners[2] = PointT(max_pt.x, max_pt.y, min_pt.z);  // bottom-right-front
wall_corners[3] = PointT(max_pt.x, max_pt.y, max_pt.z);  // top-right-front

std::cout << wall_corners[2] << std::endl;      
std::cout << wall_corners[3] << std::endl;

}




int main ()
{
pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);



pcl::PCDReader cloud_reader;
pcl::PCDWriter cloud_writer;    

pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

std::string path = "/home/abdelrahman/Wall-Painting-Robot/pcd_files/";

cloud_reader.read(path + std::string("non_plane_seg.pcd"), *cloud);


pcl::VoxelGrid<PointT> voxel_filter;  //decrese the number of points in the cloud while maintaing the shape
voxel_filter.setInputCloud(cloud);
voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
voxel_filter.filter(*voxel_cloud);

cloud_saver("voxel_cloud.pcd", path, voxel_cloud);

//Normals computation

pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
pcl::SACSegmentationFromNormals<PointT, pcl::Normal> cylinder_segmentor;
pcl::SACSegmentationFromNormals<PointT, pcl::Normal> wall_segmentor;


pcl::ModelCoefficients::Ptr cylinder_co(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr cylinder_inliers(new pcl::PointIndices);

pcl::ModelCoefficients::Ptr wall_co(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr wall_inliers(new pcl::PointIndices);


pcl::ExtractIndices<PointT> extract_indicies;
pcl::ExtractIndices<pcl::Normal> extract_indicies_temp;

pcl::PointCloud<PointT>::Ptr  extracted_cylinders(new pcl::PointCloud<PointT>);    
pcl::PointCloud<PointT>::Ptr  extracted_walls(new pcl::PointCloud<PointT>);    


normal_estimator.setSearchMethod(tree);
normal_estimator.setInputCloud(voxel_cloud);
normal_estimator.setKSearch(100); //search for the nearest neighbors
normal_estimator.compute(*cloud_normals);

// Parameters for the cylinder segmentation

cylinder_segmentor.setOptimizeCoefficients(true);
cylinder_segmentor.setModelType(pcl::SACMODEL_CYLINDER);
cylinder_segmentor.setMethodType(pcl::SAC_RANSAC);
cylinder_segmentor.setNormalDistanceWeight(0.5);
cylinder_segmentor.setMaxIterations(10000);
cylinder_segmentor.setDistanceThreshold(0.01);
cylinder_segmentor.setRadiusLimits(0.1, 0.4);

wall_segmentor.setOptimizeCoefficients(true);
wall_segmentor.setModelType(pcl::SACMODEL_NORMAL_PLANE);  // Changed to plane model
wall_segmentor.setMethodType(pcl::SAC_RANSAC);
wall_segmentor.setNormalDistanceWeight(0.2);  // Reduced for planes
wall_segmentor.setMaxIterations(10000);
wall_segmentor.setDistanceThreshold(0.05);    // Increased for walls
wall_segmentor.setAxis(Eigen::Vector3f(0, 0, 1));  // Assuming vertical walls
wall_segmentor.setEpsAngle(20.0f * (M_PI/180.0f)); // Allow 20 degrees deviation


int l = 0;

while (true)
{


wall_segmentor.setInputCloud(voxel_cloud);
wall_segmentor.setInputNormals(cloud_normals);
wall_segmentor.segment(*wall_inliers, *wall_co);


extract_indicies.setInputCloud(voxel_cloud);
extract_indicies.setIndices(wall_inliers);
extract_indicies.setNegative(false); // set to true to remove the inliers, false to keep them (extract a plane)
extract_indicies.filter(*extracted_walls); 

if (!extracted_walls->points.empty())
{   

    std::stringstream ss; ss << "ex_wall"<<l<<".pcd";

    if (extracted_walls->points.size() > 100)
    {
        cloud_saver(ss.str(),path,extracted_walls);
        std::cout << "wall " << l << " saved with size: " << extracted_walls->points.size() << std::endl;

        find_wall_boundries(extracted_walls);

        l++;
        
    }

   

    extract_indicies.setNegative(true); // set to true to remove the inliers, false to keep them (extract a plane)
    extract_indicies.filter(*voxel_cloud); // cloud will contain the cylinder points
    
    extract_indicies_temp.setInputCloud(cloud_normals);
    extract_indicies_temp.setIndices(wall_inliers);
    extract_indicies_temp.setNegative(true); // set to true to remove the inliers, false to keep them (extract a plane)
    extract_indicies_temp.filter(*cloud_normals); // cloud will contain the cylinder points

}

else {

return 0;

}



}

}