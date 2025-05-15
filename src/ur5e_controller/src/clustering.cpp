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
#include <unordered_set>
#include <string>
#include <sstream>
#include <iomanip>

using PointT = pcl::PointXYZ;

class WallDetector : public rclcpp::Node
{
public:
    WallDetector() : Node("wall_detector")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/point_cloud", rclcpp::SensorDataQoS(),
            std::bind(&WallDetector::cloud_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Wall detector node started");
        
        wall_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "detected_walls", 10);

        cropped_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "cropped_cloud", 10);    

        this->declare_parameter("crop_min_x", 0.0);
        this->declare_parameter("crop_min_y", 0.0);
        this->declare_parameter("crop_min_z", 0.0);
        this->declare_parameter("crop_max_x", 0.0);
        this->declare_parameter("crop_max_y", 0.0);
        this->declare_parameter("crop_max_z", 0.0);
        this->declare_parameter("min_wall_size", 0.0); 

        params_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&WallDetector::parametersCallback, this, std::placeholders::_1));
            
        report_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&WallDetector::report_wall_count, this));
    }
    
    int getWallCount() const { return wall_count_; }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wall_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_cloud_pub_;
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    rclcpp::TimerBase::SharedPtr report_timer_;
    
    int wall_count_ = 0;

    // Structure to track unique walls by their position and dimensions
    struct WallSignature {
        float center_x;
        float center_y;
        float center_z;
        float width;
        float height;
        
        std::string toString() const {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2);
            ss << center_x << "_" << center_y << "_" << center_z << "_" << width << "_" << height;
            return ss.str();
        }
    };
    
    // Set to store unique wall signatures
    std::unordered_set<std::string> detected_walls_;
    
    // Tolerance for considering walls as the same
    const float position_tolerance_ = 0.2f;  // 20 cm position tolerance
    const float size_tolerance_ = 0.1f;      // 10 cm size tolerance

    void report_wall_count()
    {
        RCLCPP_INFO(this->get_logger(), "Total unique walls detected: %d", wall_count_);
    }

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
            else if (param.get_name() == "min_wall_size") min_wall_size_ = param.as_double();
        }

        return result;
    }

    double min_x_ = -0.5, min_y_ = -2.0, min_z_ = 0.0;
    double max_x_ = 3.0, max_y_ = 2.0, max_z_ = 3.0;
    double min_wall_size_ = 0.5;

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg, *cloud);
    
        if (cloud->empty())
        {
            return;
        }
    
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
            return;
        }
    
        pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cropped_cloud); 
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_filter.filter(*voxel_cloud);
    
        if (voxel_cloud->empty())
        {
            return;
        }
    
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(voxel_cloud);
        normal_estimator.setKSearch(100);
        normal_estimator.compute(*cloud_normals);
    
        if (cloud_normals->empty())
        {
            return;
        }

        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.2);
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.02);
        seg.setAxis(Eigen::Vector3f(0, 0, 1));
        seg.setEpsAngle(5.0f * (M_PI / 180.0f));
        seg.setInputCloud(voxel_cloud);
        seg.setInputNormals(cloud_normals);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
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
            find_wall_boundaries(wall_cloud);
        }
    }

    void find_wall_boundaries(pcl::PointCloud<PointT>::Ptr wall_cloud)
    {
        PointT min_pt, max_pt;
        pcl::getMinMax3D(*wall_cloud, min_pt, max_pt);
        
        float wall_width = std::abs(max_pt.x - min_pt.x);
        float wall_height = std::abs(max_pt.z - min_pt.z);
        
        if (wall_width < min_wall_size_ && wall_height < min_wall_size_) {
            return;  
        }
        
        // Create a wall signature
        WallSignature signature;
        signature.center_x = (min_pt.x + max_pt.x) / 2.0f;
        signature.center_y = (min_pt.y + max_pt.y) / 2.0f;
        signature.center_z = (min_pt.z + max_pt.z) / 2.0f;
        signature.width = wall_width;
        signature.height = wall_height;
        
        // Round the signature values to reduce sensitivity
        signature.center_x = std::round(signature.center_x / position_tolerance_) * position_tolerance_;
        signature.center_y = std::round(signature.center_y / position_tolerance_) * position_tolerance_;
        signature.center_z = std::round(signature.center_z / position_tolerance_) * position_tolerance_;
        signature.width = std::round(signature.width / size_tolerance_) * size_tolerance_;
        signature.height = std::round(signature.height / size_tolerance_) * size_tolerance_;
        
        std::string wall_key = signature.toString();
        
        // Check if we've seen this wall before
        if (detected_walls_.find(wall_key) != detected_walls_.end()) {
            // Wall already detected, skip it
            return;
        }
        
        // New wall, add it to our set
        detected_walls_.insert(wall_key);
        
        geometry_msgs::msg::Point32 corners[4];
        std_msgs::msg::Float32MultiArray wall_corners;
        
        corners[0].x = min_pt.x; corners[0].y = min_pt.y; corners[0].z = min_pt.z; 
        corners[1].x = min_pt.x; corners[1].y = min_pt.y; corners[1].z = max_pt.z;
        corners[2].x = max_pt.x; corners[2].y = max_pt.y; corners[2].z = max_pt.z;
        corners[3].x = max_pt.x; corners[3].y = max_pt.y; corners[3].z = min_pt.z;

        for (int i = 0; i < 4; ++i)
        {
            wall_corners.data.push_back(corners[i].x);
            wall_corners.data.push_back(corners[i].y);
            wall_corners.data.push_back(corners[i].z);
        }
        wall_pub_->publish(wall_corners);
        
        wall_count_++;
        RCLCPP_INFO(this->get_logger(), "New wall detected (#%d) - Size: %.2fm x %.2fm at position [%.2f, %.2f, %.2f]", 
                   wall_count_, wall_width, wall_height, signature.center_x, signature.center_y, signature.center_z);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto wall_detector_node = std::make_shared<WallDetector>();
    RCLCPP_INFO(wall_detector_node->get_logger(), "Wall detector node is running. Watching for walls minimum size: %.2fm", 
                wall_detector_node->get_parameter("min_wall_size").as_double());
    rclcpp::spin(wall_detector_node);
    RCLCPP_INFO(wall_detector_node->get_logger(), "Wall detector node shutting down. Total walls detected: %d",
                wall_detector_node->getWallCount());
    rclcpp::shutdown();
    return 0;
}