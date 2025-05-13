#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <geometry_msgs/msg/point32.hpp>


using PointT = pcl::PointXYZ;

class WallDetector : public rclcpp::Node
{
public:
    WallDetector() : Node("wall_detector")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/mid360_PointCloud2", rclcpp::SensorDataQoS(),
            std::bind(&WallDetector::cloud_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Wall detector node started");
        
        wall_pub_ = this->create_publisher<geometry_msgs::msg::Point32>(
            "detected_walls", 10);
        
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr wall_pub_;

    int wall_id_ = 0;

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
            return;
        }

        // Downsample
        pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_filter.filter(*voxel_cloud);

        // Compute normals
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(voxel_cloud);
        normal_estimator.setKSearch(100);
        normal_estimator.compute(*cloud_normals);

        // Wall segmentation
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.2);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.05);
        seg.setAxis(Eigen::Vector3f(0, 0, 1));
        seg.setEpsAngle(20.0f * (M_PI / 180.0f));
        seg.setInputCloud(voxel_cloud);
        seg.setInputNormals(cloud_normals);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            RCLCPP_INFO(this->get_logger(), "No wall found");
            return;
        }

        pcl::ExtractIndices<PointT> extract;
        pcl::PointCloud<PointT>::Ptr wall_cloud(new pcl::PointCloud<PointT>);
        extract.setInputCloud(voxel_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*wall_cloud);

        if (wall_cloud->points.size() > 500)
        {
            RCLCPP_INFO(this->get_logger(), "Detected wall %d with %lu points", wall_id_, wall_cloud->size());
            find_wall_boundaries(wall_cloud);
            wall_id_++;
        }
    }

    void find_wall_boundaries(pcl::PointCloud<PointT>::Ptr wall_cloud)
    {
        PointT min_pt, max_pt;
        pcl::getMinMax3D(*wall_cloud, min_pt, max_pt);

        
        geometry_msgs::msg::Point32 corners[4];
        
        corners[0].x = min_pt.x; corners[0].y = min_pt.y; corners[0].z = min_pt.z; //
        corners[1].x = min_pt.x; corners[1].y = min_pt.y; corners[1].z = max_pt.z;
        corners[2].x = max_pt.x; corners[2].y = max_pt.y; corners[2].z = max_pt.z;
        corners[3].x = max_pt.x; corners[3].y = max_pt.y; corners[3].z = min_pt.z;

        wall_pub_->publish(corners[0]);


        RCLCPP_INFO(this->get_logger(), "Wall boundaries:");
        RCLCPP_INFO(this->get_logger(), "Min Point: [%.2f, %.2f, %.2f]", min_pt.x, min_pt.y, min_pt.z);
        RCLCPP_INFO(this->get_logger(), "Max Point: [%.2f, %.2f, %.2f]", max_pt.x, max_pt.y, max_pt.z);

        // You can publish or save this info for robot motion planning
    }
};


int main    (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallDetector>());
    rclcpp::shutdown();
    return 0;
}
