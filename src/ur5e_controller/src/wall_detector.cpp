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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using PointT = pcl::PointXYZ;

class WallDetector : public rclcpp::Node
{
public:
    WallDetector() : Node("wall_detector")
    {
        // Load config file from package or fallback path
        std::string config_path;
        try
        {
            auto pkg_share = ament_index_cpp::get_package_share_directory("ur5e_controller");
            config_path = pkg_share + "/config/config.yaml";
        }
        catch (...)
        {
            config_path = std::string(std::getenv("HOME")) + "/Wall-Painting-Robot/src/ur5e_controller/config/config.yaml";
            RCLCPP_WARN(this->get_logger(), "Could not find package share, using fallback config path: %s", config_path.c_str());
        }

        YAML::Node config;
        try
        {
            config = YAML::LoadFile(config_path);
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to load config file: %s", e.what());
            throw std::runtime_error("Could not load config.yaml");
        }

        min_x_ = -2.0;
        min_y_ = -3.0;
        min_z_ = 0.1;
        max_x_ = 0.5;
        max_y_ = -0.3;
        max_z_ = 2.0;
        min_wall_points_ = 50;
        min_wall_size_ = 0.5; // default

        if (config["lidar"] && config["lidar"]["wall_detection"])
        {
            auto wd = config["lidar"]["wall_detection"];
            if (!(wd["crop_min_x"] && wd["crop_min_y"] && wd["crop_min_z"] &&
                  wd["crop_max_x"] && wd["crop_max_y"] && wd["crop_max_z"]))
            {
                RCLCPP_FATAL(this->get_logger(), "Missing crop box values in config.yaml under lidar.wall_detection!");
                throw std::runtime_error("Missing crop box values in config.yaml");
            }
            min_x_ = wd["crop_min_x"].as<double>();
            min_y_ = wd["crop_min_y"].as<double>();
            min_z_ = wd["crop_min_z"].as<double>();
            max_x_ = wd["crop_max_x"].as<double>();
            max_y_ = wd["crop_max_y"].as<double>();
            max_z_ = wd["crop_max_z"].as<double>();
            if (wd["min_wall_points"])
            {
                min_wall_points_ = wd["min_wall_points"].as<int>();
            }
            if (wd["min_wall_size"])
            {
                min_wall_size_ = wd["min_wall_size"].as<double>();
            }
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "No lidar.wall_detection section in config.yaml!");
            throw std::runtime_error("No lidar.wall_detection section in config.yaml");
        }

        // --- New workspace bounds ---
        if (!config["painting"] || 
            !config["painting"]["workspace"] ||
            !config["painting"]["workspace"]["movement_min"] || !config["painting"]["workspace"]["movement_max"] ||
            !config["painting"]["workspace"]["approach_min"] || !config["painting"]["workspace"]["approach_max"] ||
            !config["painting"]["workspace"]["min_z"] || !config["painting"]["workspace"]["max_z"])
        {
            RCLCPP_FATAL(this->get_logger(), "Missing required painting workspace bounds in config.yaml!");
            throw std::runtime_error("Missing required painting workspace bounds in config.yaml");
        }
        
        auto painting_workspace = config["painting"]["workspace"];
        movement_min_ = painting_workspace["movement_min"].as<double>();
        movement_max_ = painting_workspace["movement_max"].as<double>();
        approach_min_ = painting_workspace["approach_min"].as<double>();
        approach_max_ = painting_workspace["approach_max"].as<double>();
        painting_min_z_ = painting_workspace["min_z"].as<double>();
        painting_max_z_ = painting_workspace["max_z"].as<double>();

        // Load scale_xy, offset_x, offset_y from yaml or use defaults
        auto painting_transform = config["painting"]["transform"];
        if (painting_transform)
        {
            scale_xy_ = painting_transform["scale_xy"] ? painting_transform["scale_xy"].as<double>() : 0.94;
            add_x_ = painting_transform["offset_x"] ? painting_transform["offset_x"].as<double>() : 0.0;
            add_y_ = painting_transform["offset_y"] ? painting_transform["offset_y"].as<double>() : -0.135;
        }
        else
        {
            scale_xy_ = 0.94;
            add_x_ = 0.0;
            add_y_ = -0.135;
        }

        RCLCPP_INFO(this->get_logger(), "Painting workspace bounds loaded from config:");
        RCLCPP_INFO(this->get_logger(), "  Movement: [%.2f, %.2f]", movement_min_, movement_max_);
        RCLCPP_INFO(this->get_logger(), "  Approach: [%.2f, %.2f]", approach_min_, approach_max_);
        RCLCPP_INFO(this->get_logger(), "  Z: [%.2f, %.2f]", painting_min_z_, painting_max_z_);
        RCLCPP_INFO(this->get_logger(), "Robot transform params: scale_xy=%.3f, offset_x=%.3f, offset_y=%.3f", scale_xy_, add_x_, add_y_);

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

        // Transformed wall marker publisher (in robot base frame)
        transformed_wall_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "transformed_wall_marker", 10);

        // Declare parameters for dynamic reconfiguration
        this->declare_parameter<double>("crop_min_x", min_x_);
        this->declare_parameter<double>("crop_min_y", min_y_);
        this->declare_parameter<double>("crop_min_z", min_z_);
        this->declare_parameter<double>("crop_max_x", max_x_);
        this->declare_parameter<double>("crop_max_y", max_y_);
        this->declare_parameter<double>("crop_max_z", max_z_);
        this->declare_parameter<int>("min_wall_points", min_wall_points_);
        this->declare_parameter<double>("min_wall_size", min_wall_size_);
        this->declare_parameter<double>("movement_min", movement_min_);
        this->declare_parameter<double>("movement_max", movement_max_);
        this->declare_parameter<double>("approach_min", approach_min_);
        this->declare_parameter<double>("approach_max", approach_max_);

        // Set initial values from parameters (overrides YAML if set)
        min_x_ = this->get_parameter("crop_min_x").as_double();
        min_y_ = this->get_parameter("crop_min_y").as_double();
        min_z_ = this->get_parameter("crop_min_z").as_double();
        max_x_ = this->get_parameter("crop_max_x").as_double();
        max_y_ = this->get_parameter("crop_max_y").as_double();
        max_z_ = this->get_parameter("crop_max_z").as_double();
        min_wall_points_ = this->get_parameter("min_wall_points").as_int();
        min_wall_size_ = this->get_parameter("min_wall_size").as_double();
        movement_min_ = this->get_parameter("movement_min").as_double();
        movement_max_ = this->get_parameter("movement_max").as_double();
        approach_min_ = this->get_parameter("approach_min").as_double();
        approach_max_ = this->get_parameter("approach_max").as_double();

        // Register callback for parameter updates
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&WallDetector::on_set_parameters, this, std::placeholders::_1));

        // TF2 buffer and listener for frame transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wall_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wall_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr transformed_wall_marker_pub_;

    double min_x_, min_y_, min_z_;
    double max_x_, max_y_, max_z_;
    int min_wall_points_;
    double min_wall_size_; // add this

    // New workspace bounds
    double movement_min_, movement_max_;
    double approach_min_, approach_max_;
    double painting_min_z_, painting_max_z_;

    // Robot reach scale and offsets (now configurable)
    double scale_xy_;
    double add_x_;
    double add_y_;

    int wall_id_ = 0;

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // TF2 buffer and listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rcl_interfaces::msg::SetParametersResult on_set_parameters(
        const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &param : params)
        {
            if (param.get_name() == "crop_min_x")
                min_x_ = param.as_double();
            else if (param.get_name() == "crop_min_y")
                min_y_ = param.as_double();
            else if (param.get_name() == "crop_min_z")
                min_z_ = param.as_double();
            else if (param.get_name() == "crop_max_x")
                max_x_ = param.as_double();
            else if (param.get_name() == "crop_max_y")
                max_y_ = param.as_double();
            else if (param.get_name() == "crop_max_z")
                max_z_ = param.as_double();
            else if (param.get_name() == "min_wall_points")
                min_wall_points_ = param.as_int();
            else if (param.get_name() == "min_wall_size")
                min_wall_size_ = param.as_double();
            else if (param.get_name() == "movement_min")
                movement_min_ = param.as_double();
            else if (param.get_name() == "movement_max")
                movement_max_ = param.as_double();
            else if (param.get_name() == "approach_min")
                approach_min_ = param.as_double();
            else if (param.get_name() == "approach_max")
                approach_max_ = param.as_double();
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Crop box and wall detection parameters updated";
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
        min_wall_size_ = this->get_parameter("min_wall_size").as_double();
        movement_min_ = this->get_parameter("movement_min").as_double();
        movement_max_ = this->get_parameter("movement_max").as_double();
        approach_min_ = this->get_parameter("approach_min").as_double();
        approach_max_ = this->get_parameter("approach_max").as_double();

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

    // Clamp a point's x/y/z to min/max bounds based on wall orientation
    geometry_msgs::msg::Point boundPointDynamic(
        const geometry_msgs::msg::Point &pt,
        const Eigen::Vector3f &wall_normal)
    {
        geometry_msgs::msg::Point out = pt;

        // Determine approach and movement axes: 0=x, 1=y
        int approach_axis = (std::abs(wall_normal.y()) > std::abs(wall_normal.x())) ? 1 : 0;
        int movement_axis = 1 - approach_axis;

        // Axis pointers for easier access
        double *coords[2] = {&out.x, &out.y};
        const double min_vals[2] = {approach_min_, movement_min_};
        const double max_vals[2] = {approach_max_, movement_max_};

        // Clamp approach axis
        if (*coords[approach_axis] < min_vals[0]) *coords[approach_axis] = min_vals[0];
        if (*coords[approach_axis] > max_vals[0]) *coords[approach_axis] = max_vals[0];

        // Clamp movement axis
        if (*coords[movement_axis] < min_vals[1]) *coords[movement_axis] = min_vals[1];
        if (*coords[movement_axis] > max_vals[1]) *coords[movement_axis] = max_vals[1];

        // Clamp z
        if (out.z < painting_min_z_) out.z = painting_min_z_;
        if (out.z > painting_max_z_) out.z = painting_max_z_;

        return out;
    }

    // Apply robot-specific transformations to a point (dynamic axis)
    geometry_msgs::msg::Point applyRobotTransformsDynamic(
        const geometry_msgs::msg::Point &pt,
        const Eigen::Vector3f &wall_normal)
    {
        geometry_msgs::msg::Point result = pt;
        result = boundPointDynamic(result, wall_normal);

        // Scale x and y toward zero to bring points closer to robot base
        result.x *= scale_xy_;
        result.y *= scale_xy_;

        // Offset x and y after scaling to adjust for robot's reach
        result.x += add_x_;
        result.y += add_y_;

        // Apply bounds again to ensure point is within robot's workspace
        result = boundPointDynamic(result, wall_normal);

        return result;
    }

    // Transform a point from livox_frame to base frame
    geometry_msgs::msg::Point transformToBase(const geometry_msgs::msg::Point &pt_in_lidar)
    {
        geometry_msgs::msg::PointStamped input, output;
        input.header.frame_id = "livox_frame";
        input.header.stamp = this->now();
        input.point = pt_in_lidar;
        try
        {
            // Wait for transform to be available
            if (!tf_buffer_->canTransform("livox_frame", "base", tf2::TimePointZero, tf2::durationFromSec(1.0)))
            {
                RCLCPP_ERROR(this->get_logger(), "Transform from livox_frame to base not available!");
                return pt_in_lidar;
            }
            output = tf_buffer_->transform(input, "base", tf2::durationFromSec(1.0));
            return output.point;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "TF transform failed: %s", ex.what());
            return pt_in_lidar;
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
        for (const auto &pt : wall_cloud->points)
        {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            Eigen::Vector3f vec = p - plane_point;
            float u_coord = vec.dot(u);
            float v_coord = vec.dot(v);
            uv_coords.emplace_back(u_coord, v_coord);
            if (u_coord < min_u)
                min_u = u_coord;
            if (u_coord > max_u)
                max_u = u_coord;
            if (v_coord < min_v)
                min_v = v_coord;
            if (v_coord > max_v)
                max_v = v_coord;
        }

        // Compute 4 corners in (u,v), then map back to 3D
        std::vector<Eigen::Vector3f> corners_3d;
        std::vector<std::pair<float, float>> uv_box = {
            {min_u, min_v}, {min_u, max_v}, {max_u, max_v}, {max_u, min_v}};
        for (const auto &uv : uv_box)
        {
            Eigen::Vector3f corner = plane_point + u * uv.first + v * uv.second;
            corners_3d.push_back(corner);
        }

        // --- Compute wall real-world size and filter by min_wall_size_ ---
        if (corners_3d.size() == 4)
        {
            // Compute width (distance between [0] and [3]), height (distance between [0] and [1])
            double width = (corners_3d[0] - corners_3d[3]).norm();
            double height = (corners_3d[0] - corners_3d[1]).norm();
            if (width < min_wall_size_ || height < min_wall_size_)
            {
                RCLCPP_INFO(this->get_logger(), "Wall rejected: size too small (width=%.3f, height=%.3f, min=%.3f)", width, height, min_wall_size_);
                return;
            }
        }

        // Transform corners from lidar frame to robot base frame before robot-specific transforms
        std::vector<geometry_msgs::msg::Point> transformed_corners;
        for (const auto &c : corners_3d)
        {
            geometry_msgs::msg::Point corner_point;
            corner_point.x = c.x();
            corner_point.y = c.y();
            corner_point.z = c.z();

            // Transform to robot base frame
            geometry_msgs::msg::Point base_point = transformToBase(corner_point);

            // Apply robot-specific transformations (scaling, offset, bounds) using dynamic axis logic
            geometry_msgs::msg::Point transformed_point = applyRobotTransformsDynamic(base_point, plane_normal);
            transformed_corners.push_back(transformed_point);
        }

        // --- Sort corners: first 2 are top (highest z), right to left (descending x), last 2 are bottom (lowest z), right to left (descending x) ---
        std::vector<geometry_msgs::msg::Point> viz_transformed_corners = transformed_corners; // Keep original order for viz
        if (transformed_corners.size() == 4)
        {
            // Find top and bottom indices
            std::vector<size_t> top_indices, bottom_indices;
            double max_z = std::max({transformed_corners[0].z, transformed_corners[1].z, transformed_corners[2].z, transformed_corners[3].z});
            double min_z = std::min({transformed_corners[0].z, transformed_corners[1].z, transformed_corners[2].z, transformed_corners[3].z});
            for (size_t i = 0; i < 4; ++i)
            {
                if (std::abs(transformed_corners[i].z - max_z) < 1e-4)
                    top_indices.push_back(i);
                else if (std::abs(transformed_corners[i].z - min_z) < 1e-4)
                    bottom_indices.push_back(i);
            }
            // Sort top by descending x (right to left)
            std::sort(top_indices.begin(), top_indices.end(), [&](size_t a, size_t b)
                      { return transformed_corners[a].x > transformed_corners[b].x; });
            // Sort bottom by descending x (right to left)
            std::sort(bottom_indices.begin(), bottom_indices.end(), [&](size_t a, size_t b)
                      { return transformed_corners[a].x > transformed_corners[b].x; });
            // Compose sorted_corners: [top_right, top_left, bottom_right, bottom_left]
            std::vector<geometry_msgs::msg::Point> sorted_corners;
            for (auto idx : top_indices)
                sorted_corners.push_back(transformed_corners[idx]);
            for (auto idx : bottom_indices)
                sorted_corners.push_back(transformed_corners[idx]);
            transformed_corners = sorted_corners;
        }

        // Publish transformed wall corners
        std_msgs::msg::Float32MultiArray wall_corners;
        for (const auto &c : transformed_corners)
        {
            wall_corners.data.push_back(c.x);
            wall_corners.data.push_back(c.y);
            wall_corners.data.push_back(c.z);
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

        // Add the 4 corners to visualization using the ORIGINAL corners
        // (not transformed) because the visualization is in the lidar frame
        for (const auto &c : corners_3d)
        {
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

        // Publish transformed wall marker for robot base frame visualization
        visualization_msgs::msg::Marker transformed_marker;
        transformed_marker.header.frame_id = "base"; 
        transformed_marker.header.stamp = this->now();
        transformed_marker.ns = "transformed_walls";
        transformed_marker.id = wall_id_;
        transformed_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        transformed_marker.action = visualization_msgs::msg::Marker::ADD;
        transformed_marker.scale.x = 0.05;
        transformed_marker.color.r = 0.0f;
        transformed_marker.color.g = 1.0f;
        transformed_marker.color.b = 1.0f;
        transformed_marker.color.a = 1.0f;
        transformed_marker.lifetime = rclcpp::Duration::from_seconds(2.0);

        // Add the 4 transformed corners to visualization (using original order)
        for (const auto &c : viz_transformed_corners)
        {
            transformed_marker.points.push_back(c);
        }
        // Close the border
        transformed_marker.points.push_back(viz_transformed_corners[0]);

        transformed_wall_marker_pub_->publish(transformed_marker);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallDetector>());
    rclcpp::shutdown();
    return 0;
}
