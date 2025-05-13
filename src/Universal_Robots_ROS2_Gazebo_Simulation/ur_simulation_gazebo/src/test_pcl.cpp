#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>          
#include <geometry_msgs/msg/polygon_stamped.hpp>

class WallDetectionNode : public rclcpp::Node
{
public:
    WallDetectionNode() : Node("wall_detection")
    {
        // Subscribers
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/mid360_PointCloud2", 10,
            std::bind(&WallDetectionNode::cloud_callback, this, std::placeholders::_1));

        // Publishers
        voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "voxelized_cloud", 10);
        wall_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "detected_walls", 10);
        wall_polygon_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "wall_polygon", 10);
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // 1. Voxel Grid Filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_filter.filter(*voxel_cloud);

        // Publish voxelized cloud
        sensor_msgs::msg::PointCloud2 voxel_msg;
        pcl::toROSMsg(*voxel_cloud, voxel_msg);
        voxel_msg.header = msg->header;
        voxel_pub_->publish(voxel_msg);

        // 2. Normal Estimation
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(voxel_cloud);
        normal_estimator.setKSearch(50);
        normal_estimator.compute(*cloud_normals);

        // 3. Wall Segmentation
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> wall_segmentor;
        pcl::ModelCoefficients::Ptr wall_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr wall_inliers(new pcl::PointIndices);

        wall_segmentor.setOptimizeCoefficients(true);
        wall_segmentor.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        wall_segmentor.setMethodType(pcl::SAC_RANSAC);
        wall_segmentor.setNormalDistanceWeight(0.2);
        wall_segmentor.setMaxIterations(1000);
        wall_segmentor.setDistanceThreshold(0.05);
        wall_segmentor.setInputCloud(voxel_cloud);
        wall_segmentor.setInputNormals(cloud_normals);
        wall_segmentor.segment(*wall_inliers, *wall_coefficients);

        if (!wall_inliers->indices.empty())
        {
            // Extract and publish wall points
            pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(voxel_cloud);
            extract.setIndices(wall_inliers);
            extract.setNegative(false);
            extract.filter(*wall_cloud);

            sensor_msgs::msg::PointCloud2 wall_msg;
            pcl::toROSMsg(*wall_cloud, wall_msg);
            wall_msg.header = msg->header;
            wall_pub_->publish(wall_msg);

            // Find and publish wall boundaries
            publishWallPolygon(wall_cloud, msg->header);
        }
    }

    void publishWallPolygon(const pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud,
                           const std_msgs::msg::Header& header)
    {
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*wall_cloud, min_pt, max_pt);

        geometry_msgs::msg::PolygonStamped polygon;
        polygon.header = header;

        // Create corner points
        geometry_msgs::msg::Point32 corners[4];
        corners[0].x = min_pt.x; corners[0].y = min_pt.y; corners[0].z = min_pt.z;
        corners[1].x = min_pt.x; corners[1].y = min_pt.y; corners[1].z = max_pt.z;
        corners[2].x = max_pt.x; corners[2].y = max_pt.y; corners[2].z = max_pt.z;
        corners[3].x = max_pt.x; corners[3].y = max_pt.y; corners[3].z = min_pt.z;

        for (const auto& corner : corners) {
            polygon.polygon.points.push_back(corner);
        }

        wall_polygon_pub_->publish(polygon);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr wall_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr wall_polygon_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}