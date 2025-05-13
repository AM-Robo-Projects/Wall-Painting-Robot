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
#include <pcl/filters/crop_box.h>
#include <std_msgs/msg/float32_multi_array.hpp>


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
        
        wall_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "detected_walls", 10);

        cropped_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "cropped_cloud", 10);    

        this->declare_parameter("crop_min_x", -700.0);
        this->declare_parameter("crop_min_y", -500.0);
        this->declare_parameter("crop_min_z", 0.0);
        this->declare_parameter("crop_max_x", 900.0);
        this->declare_parameter("crop_max_y", -900.0);
        this->declare_parameter("crop_max_z", 3.0);

        // Parameter change callback
        params_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&WallDetector::parametersCallback, this, std::placeholders::_1));
        
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wall_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_cloud_pub_;
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters)
        {
            if (param.get_name() == "min_x") min_x_ = param.as_double();
            else if (param.get_name() == "min_y") min_y_ = param.as_double();
            else if (param.get_name() == "min_z") min_z_ = param.as_double();
            else if (param.get_name() == "max_x") max_x_ = param.as_double();
            else if (param.get_name() == "max_y") max_y_ = param.as_double();
            else if (param.get_name() == "max_z") max_z_ = param.as_double();
        }

        return result;
    }

    double min_x_ = -0.5, min_y_ = -2.0, min_z_ = 0.0;
    double max_x_ = 3.0, max_y_ = 2.0, max_z_ = 3.0;


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
    
        // Crop box filter first
        pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>);
        pcl::CropBox<PointT> crop_box;
    
        crop_box.setInputCloud(cloud); 
        crop_box.setNegative(false);
    

        crop_box.setMin(Eigen::Vector4f(min_x_, min_y_, min_z_, 1.0));
        crop_box.setMax(Eigen::Vector4f(max_x_, max_y_, max_z_, 1.0));   
    
        crop_box.filter(*cropped_cloud);

        sensor_msgs::msg::PointCloud2 cropped_msg;
        pcl::toROSMsg(*cropped_cloud, cropped_msg);
        cropped_msg.header = msg->header;
        cropped_cloud_pub_->publish(cropped_msg);


        if (cropped_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Cropped point cloud is empty");
            return;
        }
    
        // apply voxel downsampling
        pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cropped_cloud); 
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_filter.filter(*voxel_cloud);
    
        if (voxel_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Voxel filtered cloud is empty");
            return;
        }
    
        // Compute normals
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(voxel_cloud);
        normal_estimator.setKSearch(100);
        normal_estimator.compute(*cloud_normals);
    
        if (cloud_normals->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Normal estimation returned empty cloud");
            return;
        }

        // Wall segmentation
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.2);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.02); // How far points from a plane to be considered inliers
        seg.setAxis(Eigen::Vector3f(0, 0, 1));
        seg.setEpsAngle(5.0f * (M_PI / 180.0f));
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

        if (wall_cloud->points.size() > 100)
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
        std_msgs::msg::Float32MultiArray wall_corners;
        
        corners[0].x = min_pt.x; corners[0].y = min_pt.y; corners[0].z = min_pt.z; 
        corners[1].x = min_pt.x; corners[1].y = min_pt.y; corners[1].z = max_pt.z;
        corners[2].x = max_pt.x; corners[2].y = max_pt.y; corners[2].z = max_pt.z;
        corners[3].x = max_pt.x; corners[3].y = max_pt.y; corners[3].z = min_pt.z;

        // Publish the wall corner
        for (int i = 0; i < 4; ++i)
        {
            wall_corners.data.push_back(corners[i].x);
            wall_corners.data.push_back(corners[i].y);
            wall_corners.data.push_back(corners[i].z);

        }
        wall_pub_->publish(wall_corners);



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
