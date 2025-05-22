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
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <string>
#include <visualization_msgs/msg/marker.hpp>


using PointT = pcl::PointXYZ;


class WallDetector : public rclcpp::Node
{
public:
    WallDetector() : Node("wall_detector")
    {
        // Load config file from package or fallback path
        std::string config_path;
        try {
            auto pkg_share = ament_index_cpp::get_package_share_directory("ur5e_controller");
            config_path = pkg_share + "/config/lidar_config.yaml";
        } catch (...) {
            config_path = std::string(std::getenv("HOME")) + "/Wall-Painting-Robot/src/ur5e_controller/config/lidar_config.yaml";
            RCLCPP_WARN(this->get_logger(), "Could not find package share, using fallback config path: %s", config_path.c_str());
        }

        YAML::Node config;
        try {
            config = YAML::LoadFile(config_path);
        } catch (const std::exception &e) {
            RCLCPP_FATAL(this->get_logger(), "BIG ERROR: Failed to load config file: %s", e.what());
            throw std::runtime_error("Could not load lidar_config.yaml");
        }

        // Default crop box and wall detection values
        min_x_ = -2.0; min_y_ = -3.0; min_z_ = 0.1;
        max_x_ = 0.5; max_y_ = -0.3; max_z_ = 2.0;
        min_wall_points_ = 50;

        if (config["wall_detection"]) {
            auto wd = config["wall_detection"];
            // Check for all required crop box values
            if (!(wd["crop_min_x"] && wd["crop_min_y"] && wd["crop_min_z"] &&
                  wd["crop_max_x"] && wd["crop_max_y"] && wd["crop_max_z"])) {
                RCLCPP_FATAL(this->get_logger(), "BIG ERROR: Missing crop box values in lidar_config.yaml under wall_detection!");
                throw std::runtime_error("Missing crop box values in lidar_config.yaml");
            }
            min_x_ = wd["crop_min_x"].as<double>();
            min_y_ = wd["crop_min_y"].as<double>();
            min_z_ = wd["crop_min_z"].as<double>();
            max_x_ = wd["crop_max_x"].as<double>();
            max_y_ = wd["crop_max_y"].as<double>();
            max_z_ = wd["crop_max_z"].as<double>();
            if (wd["min_wall_points"]) {
                min_wall_points_ = wd["min_wall_points"].as<int>();
            }
        } else {
            RCLCPP_FATAL(this->get_logger(), "BIG ERROR: No wall_detection section in lidar_config.yaml!");
            throw std::runtime_error("No wall_detection section in lidar_config.yaml");
        }

        // Use correct topic from config (as in livox_converter.py)
        std::string point_cloud_topic = "/livox/point_cloud";

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            point_cloud_topic, rclcpp::SensorDataQoS(),
            std::bind(&WallDetector::cloud_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Wall detector node started (subscribing to %s)", point_cloud_topic.c_str());
        
        wall_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "detected_walls", 10);

        cropped_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "cropped_cloud", 10);    

        // Visualization marker publisher
        wall_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "wall_marker", 10);

        // Declare parameters for dynamic reconfiguration
        this->declare_parameter<double>("crop_min_x", min_x_);
        this->declare_parameter<double>("crop_min_y", min_y_);
        this->declare_parameter<double>("crop_min_z", min_z_);
        this->declare_parameter<double>("crop_max_x", max_x_);
        this->declare_parameter<double>("crop_max_y", max_y_);
        this->declare_parameter<double>("crop_max_z", max_z_);
        this->declare_parameter<int>("min_wall_points", min_wall_points_);

        // Set initial values from parameters (overrides YAML if set)
        min_x_ = this->get_parameter("crop_min_x").as_double();
        min_y_ = this->get_parameter("crop_min_y").as_double();
        min_z_ = this->get_parameter("crop_min_z").as_double();
        max_x_ = this->get_parameter("crop_max_x").as_double();
        max_y_ = this->get_parameter("crop_max_y").as_double();
        max_z_ = this->get_parameter("crop_max_z").as_double();
        min_wall_points_ = this->get_parameter("min_wall_points").as_int();

        // Register callback for parameter updates
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&WallDetector::on_set_parameters, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wall_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wall_marker_pub_;

    double min_x_, min_y_, min_z_;
    double max_x_, max_y_, max_z_;
    int min_wall_points_;

    int wall_id_ = 0;

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rcl_interfaces::msg::SetParametersResult on_set_parameters(
        const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &param : params) {
            if (param.get_name() == "crop_min_x") min_x_ = param.as_double();
            else if (param.get_name() == "crop_min_y") min_y_ = param.as_double();
            else if (param.get_name() == "crop_min_z") min_z_ = param.as_double();
            else if (param.get_name() == "crop_max_x") max_x_ = param.as_double();
            else if (param.get_name() == "crop_max_y") max_y_ = param.as_double();
            else if (param.get_name() == "crop_max_z") max_z_ = param.as_double();
            else if (param.get_name() == "min_wall_points") min_wall_points_ = param.as_int();
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Crop box and wall detection parameters updated";
        RCLCPP_INFO(this->get_logger(), "Crop box parameters updated: min(%.2f, %.2f, %.2f) max(%.2f, %.2f, %.2f), min_wall_points: %d",
            min_x_, min_y_, min_z_, max_x_, max_y_, max_z_, min_wall_points_);
        return result;
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Always get latest parameter values
        min_x_ = this->get_parameter("crop_min_x").as_double();
        min_y_ = this->get_parameter("crop_min_y").as_double();
        min_z_ = this->get_parameter("crop_min_z").as_double();
        max_x_ = this->get_parameter("crop_max_x").as_double();
        max_y_ = this->get_parameter("crop_max_y").as_double();
        max_z_ = this->get_parameter("crop_max_z").as_double();
        min_wall_points_ = this->get_parameter("min_wall_points").as_int();
    
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty())
            return;
    
        // Crop box filter
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
            return;
    
        // Voxel downsampling
        pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cropped_cloud); 
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_filter.filter(*voxel_cloud);

        if (voxel_cloud->empty())
            return;
    
        // Compute normals
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(voxel_cloud);
        normal_estimator.setKSearch(100);
        normal_estimator.compute(*cloud_normals);

        if (cloud_normals->empty())
            return;

        // Wall segmentation using RANSAC with normals
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
            return;

        // Extract wall points
        pcl::ExtractIndices<PointT> extract;
        pcl::PointCloud<PointT>::Ptr wall_cloud(new pcl::PointCloud<PointT>);
        extract.setInputCloud(voxel_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*wall_cloud);

        if (static_cast<int>(wall_cloud->points.size()) >= min_wall_points_)
        {
            find_wall_boundaries(wall_cloud, coefficients);
            wall_id_++;
        }
    }

    // Find wall boundaries and publish marker and corners
    void find_wall_boundaries(pcl::PointCloud<PointT>::Ptr wall_cloud, pcl::ModelCoefficients::Ptr coefficients)
    {
        // Plane coefficients: ax + by + cz + d = 0
        Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        plane_normal.normalize();

        float d = coefficients->values[3];
        Eigen::Vector3f plane_point = -d * plane_normal;

        // Create local axes (u,v) on the plane
        Eigen::Vector3f u = plane_normal.unitOrthogonal();
        Eigen::Vector3f v = plane_normal.cross(u);

        // Project all points to plane, get their (u,v) coordinates
        float min_u = std::numeric_limits<float>::max(), max_u = -std::numeric_limits<float>::max();
        float min_v = std::numeric_limits<float>::max(), max_v = -std::numeric_limits<float>::max();

        std::vector<Eigen::Vector2f> uv_coords;
        for (const auto& pt : wall_cloud->points) {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            Eigen::Vector3f vec = p - plane_point;
            float u_coord = vec.dot(u);
            float v_coord = vec.dot(v);
            uv_coords.emplace_back(u_coord, v_coord);
            if (u_coord < min_u) min_u = u_coord;
            if (u_coord > max_u) max_u = u_coord;
            if (v_coord < min_v) min_v = v_coord;
            if (v_coord > max_v) max_v = v_coord;
        }

        // Compute 4 corners in (u,v), then map back to 3D
        std::vector<Eigen::Vector3f> corners_3d;
        std::vector<std::pair<float, float>> uv_box = {
            {min_u, min_v}, {min_u, max_v}, {max_u, max_v}, {max_u, min_v}
        };
        for (const auto& uv : uv_box) {
            Eigen::Vector3f corner = plane_point + u * uv.first + v * uv.second;
            corners_3d.push_back(corner);
        }

        // Publish wall corners
        std_msgs::msg::Float32MultiArray wall_corners;
        for (const auto& c : corners_3d) {
            wall_corners.data.push_back(c.x());
            wall_corners.data.push_back(c.y());
            wall_corners.data.push_back(c.z());
        }
        wall_pub_->publish(wall_corners);

        // Publish visualization marker for RViz
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "livox_frame";
        marker.header.stamp = this->now();
        marker.ns = "walls";
        marker.id = wall_id_;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.03;
        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        marker.lifetime = rclcpp::Duration::from_seconds(2.0);

        // Add the 4 corners, then repeat the first to close the loop
        for (const auto& c : corners_3d) {
            geometry_msgs::msg::Point p;
            p.x = c.x();
            p.y = c.y();
            p.z = c.z();
            marker.points.push_back(p);
        }
        // Close the border
        geometry_msgs::msg::Point p;
        p.x = corners_3d[0].x();
        p.y = corners_3d[0].y();
        p.z = corners_3d[0].z();
        marker.points.push_back(p);

        wall_marker_pub_->publish(marker);
    }
};


int main    (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallDetector>());
    rclcpp::shutdown();
    return 0;
}
