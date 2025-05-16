#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <geometry_msgs/msg/point32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
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
        loadConfig();
        setupCommunications();
        printConfig();
        
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(publish_interval_),
            std::bind(&WallDetector::publish_walls_and_reset, this));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr walls_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    double min_x_ = -0.5, min_y_ = -2.0, min_z_ = 0.0;
    double max_x_ = 3.0, max_y_ = 2.0, max_z_ = 3.0;
    double min_wall_size_ = 0.5;
    double publish_interval_ = 10.0;
    int rolling_window_size_ = 3;
    bool enable_wall_merging_ = true;
    
    int wall_count_ = 0;
    
    struct Wall {
        float center_x;
        float center_y;
        float center_z;
        float width;
        float height;
        geometry_msgs::msg::Point32 corners[4];
    };
    
    std::vector<Wall> current_walls_;
    std::vector<Wall> previous_walls_;
    std::unordered_set<std::string> current_scan_walls_;
    std::vector<Wall> prev_smoothed_walls_;
    std::unordered_set<int> previous_marker_ids_;
    
    const float position_tolerance_ = 0.2f;
    const float size_tolerance_ = 0.1f;

    void loadConfig() {
        this->declare_parameter("config_path", "");
        std::string config_path = this->get_parameter("config_path").as_string();
        
        if (config_path.empty()) {
            try {
                std::string package_path = ament_index_cpp::get_package_share_directory("ur5e_controller");
                config_path = package_path + "/config/lidar_config.yaml";
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to find package share directory: %s", e.what());
                config_path = "";
            }
        }
        
        if (!config_path.empty()) {
            loadYamlConfig(config_path);
        }
        
        this->declare_parameter("crop_min_x", min_x_);
        this->declare_parameter("crop_min_y", min_y_);
        this->declare_parameter("crop_min_z", min_z_);
        this->declare_parameter("crop_max_x", max_x_);
        this->declare_parameter("crop_max_y", max_y_);
        this->declare_parameter("crop_max_z", max_z_);
        this->declare_parameter("min_wall_size", min_wall_size_);
        this->declare_parameter("publish_interval", publish_interval_);
        this->declare_parameter("rolling_window_size", rolling_window_size_);
        this->declare_parameter("enable_wall_merging", enable_wall_merging_);
        
        if (this->has_parameter("crop_min_x")) min_x_ = this->get_parameter("crop_min_x").as_double();
        if (this->has_parameter("crop_min_y")) min_y_ = this->get_parameter("crop_min_y").as_double();
        if (this->has_parameter("crop_min_z")) min_z_ = this->get_parameter("crop_min_z").as_double();
        if (this->has_parameter("crop_max_x")) max_x_ = this->get_parameter("crop_max_x").as_double();
        if (this->has_parameter("crop_max_y")) max_y_ = this->get_parameter("crop_max_y").as_double();
        if (this->has_parameter("crop_max_z")) max_z_ = this->get_parameter("crop_max_z").as_double();
        if (this->has_parameter("min_wall_size")) min_wall_size_ = this->get_parameter("min_wall_size").as_double();
        if (this->has_parameter("publish_interval")) publish_interval_ = this->get_parameter("publish_interval").as_double();
        if (this->has_parameter("rolling_window_size")) rolling_window_size_ = this->get_parameter("rolling_window_size").as_int();
        if (this->has_parameter("enable_wall_merging")) enable_wall_merging_ = this->get_parameter("enable_wall_merging").as_bool();
    }
    
    void setupCommunications() {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/point_cloud", rclcpp::SensorDataQoS(),
            std::bind(&WallDetector::cloud_callback, this, std::placeholders::_1));

        walls_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "all_walls", 10);

        cropped_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "cropped_cloud", 10);    

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "wall_markers", 10);

        params_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&WallDetector::parametersCallback, this, std::placeholders::_1));
    }
    
    void printConfig() {
        RCLCPP_INFO(this->get_logger(), "==== Wall Detector Node Started ====");
        RCLCPP_INFO(this->get_logger(), "Configuration parameters:");
        RCLCPP_INFO(this->get_logger(), "  Crop box min: [%.2f, %.2f, %.2f]", min_x_, min_y_, min_z_);
        RCLCPP_INFO(this->get_logger(), "  Crop box max: [%.2f, %.2f, %.2f]", max_x_, max_y_, max_z_);
        RCLCPP_INFO(this->get_logger(), "  Min wall size: %.2f m (real-world measurement)", min_wall_size_);
        RCLCPP_INFO(this->get_logger(), "  Wall merging: %s", enable_wall_merging_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "  Publish interval: %.2f s", publish_interval_);
        RCLCPP_INFO(this->get_logger(), "  Rolling window: %s (size: %d cycles)", 
                   rolling_window_size_ > 0 ? "enabled" : "disabled", 
                   rolling_window_size_ > 0 ? rolling_window_size_ : 0);
        RCLCPP_INFO(this->get_logger(), "  Listening on topic: /livox/point_cloud");
        RCLCPP_INFO(this->get_logger(), "  Publishing all walls to: all_walls");
        RCLCPP_INFO(this->get_logger(), "  Publishing cropped cloud to: cropped_cloud");
        RCLCPP_INFO(this->get_logger(), "  Publishing visualization markers to: wall_markers");
        RCLCPP_INFO(this->get_logger(), "====================================");
    }

    void loadYamlConfig(const std::string& config_path) {
        try {
            RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_path.c_str());
            YAML::Node config = YAML::LoadFile(config_path);
            
            if (config["wall_detection"]) {
                auto wall_config = config["wall_detection"];
                if (wall_config["crop_min_x"]) min_x_ = wall_config["crop_min_x"].as<double>();
                if (wall_config["crop_min_y"]) min_y_ = wall_config["crop_min_y"].as<double>();
                if (wall_config["crop_min_z"]) min_z_ = wall_config["crop_min_z"].as<double>();
                if (wall_config["crop_max_x"]) max_x_ = wall_config["crop_max_x"].as<double>();
                if (wall_config["crop_max_y"]) max_y_ = wall_config["crop_max_y"].as<double>();
                if (wall_config["crop_max_z"]) max_z_ = wall_config["crop_max_z"].as<double>();
                if (wall_config["min_wall_size"]) min_wall_size_ = wall_config["min_wall_size"].as<double>();
                if (wall_config["publish_interval"]) publish_interval_ = wall_config["publish_interval"].as<double>();
                if (wall_config["rolling_window_size"]) rolling_window_size_ = wall_config["rolling_window_size"].as<int>();
                if (wall_config["enable_wall_merging"]) enable_wall_merging_ = wall_config["enable_wall_merging"].as<bool>();
            } else {
                RCLCPP_WARN(this->get_logger(), "No 'wall_detection' section found in config file");
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing YAML config: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading config file: %s", e.what());
        }
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters)
        {
            if (param.get_name() == "crop_min_x") min_x_ = param.as_double();
            else if (param.get_name() == "crop_min_y") min_y_ = param.as_double();
            else if (param.get_name() == "crop_min_z") min_z_ = param.as_double();
            else if (param.get_name() == "crop_max_x") max_x_ = param.as_double();
            else if (param.get_name() == "crop_max_y") max_y_ = param.as_double();
            else if (param.get_name() == "crop_max_z") max_z_ = param.as_double();
            else if (param.get_name() == "min_wall_size") 
                min_wall_size_ = param.as_double();
            else if (param.get_name() == "publish_interval") {
                publish_interval_ = param.as_double();
                publish_timer_->cancel();
                publish_timer_ = this->create_wall_timer(
                    std::chrono::duration<double>(publish_interval_),
                    std::bind(&WallDetector::publish_walls_and_reset, this));
            }
            else if (param.get_name() == "rolling_window_size") rolling_window_size_ = param.as_int();
            else if (param.get_name() == "enable_wall_merging") enable_wall_merging_ = param.as_bool();
        }

        return result;
    }
    
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg, *cloud);
    
        if (cloud->empty()) {
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

        if (cropped_cloud->empty()) {
            return;
        }
    
        pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cropped_cloud); 
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_filter.filter(*voxel_cloud);
    
        if (voxel_cloud->empty()) {
            return;
        }
    
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(voxel_cloud);
        normal_estimator.setKSearch(100);
        normal_estimator.compute(*cloud_normals);
    
        if (cloud_normals->empty()) {
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

        if (inliers->indices.empty()) {
            return;
        }

        pcl::ExtractIndices<PointT> extract;
        pcl::PointCloud<PointT>::Ptr wall_cloud(new pcl::PointCloud<PointT>);
        extract.setInputCloud(voxel_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*wall_cloud);

        if (wall_cloud->points.size() > 100) {
            find_wall_boundaries(wall_cloud);
        }
    }

    float calculateEuclideanDistance(const geometry_msgs::msg::Point32& p1, const geometry_msgs::msg::Point32& p2) {
        float dx = p1.x - p2.x;
        float dy = p1.y - p2.y;
        float dz = p1.z - p2.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    void calculateTrueDimensions(const Wall& wall, float& true_width, float& true_height) {
        float width1 = calculateEuclideanDistance(wall.corners[0], wall.corners[3]);
        float width2 = calculateEuclideanDistance(wall.corners[1], wall.corners[2]);
        true_width = std::max(width1, width2);
        
        float height1 = calculateEuclideanDistance(wall.corners[0], wall.corners[1]);
        float height2 = calculateEuclideanDistance(wall.corners[3], wall.corners[2]);
        true_height = std::max(height1, height2);
    }

    void find_wall_boundaries(pcl::PointCloud<PointT>::Ptr wall_cloud)
    {
        PointT min_pt, max_pt;
        pcl::getMinMax3D(*wall_cloud, min_pt, max_pt);
        
        float center_x = (min_pt.x + max_pt.x) / 2.0f;
        float center_y = (min_pt.y + max_pt.y) / 2.0f;
        float center_z = (min_pt.z + max_pt.z) / 2.0f;
        
        Wall wall;
        wall.center_x = center_x;
        wall.center_y = center_y;
        wall.center_z = center_z;
        
        wall.corners[0].x = min_pt.x; wall.corners[0].y = min_pt.y; wall.corners[0].z = min_pt.z; 
        wall.corners[1].x = min_pt.x; wall.corners[1].y = min_pt.y; wall.corners[1].z = max_pt.z;
        wall.corners[2].x = max_pt.x; wall.corners[2].y = max_pt.y; wall.corners[2].z = max_pt.z;
        wall.corners[3].x = max_pt.x; wall.corners[3].y = max_pt.y; wall.corners[3].z = min_pt.z;
        
        float true_width, true_height;
        calculateTrueDimensions(wall, true_width, true_height);
        
        wall.width = true_width;
        wall.height = true_height;
        
        if (true_width < min_wall_size_ || true_height < min_wall_size_)
            return;
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << wall.center_x << "_" << wall.center_y << "_" << wall.center_z;
        std::string wall_key = ss.str();
        
        if (current_scan_walls_.find(wall_key) != current_scan_walls_.end()) {
            return;
        }
        
        bool merged = false;
        if (enable_wall_merging_) {
            for (auto& existing_wall : current_walls_) {
                if (should_merge_walls(wall, existing_wall)) {
                    Wall merged_wall = merge_walls(wall, existing_wall);
                    existing_wall = merged_wall; 
                    merged = true;
                    break;
                }
            }
        }
        
        if (!merged) {
            current_scan_walls_.insert(wall_key);
            current_walls_.push_back(wall);
        }
    }
    
    void publish_walls_and_reset()
    {
        std::vector<Wall> walls_to_publish = current_walls_;
        
        if (rolling_window_size_ > 0 && !previous_walls_.empty()) {
            for (const auto& prev_wall : previous_walls_) {
                bool merged = false;
                
                if (enable_wall_merging_) {
                    for (auto& existing_wall : walls_to_publish) {
                        if (should_merge_walls(prev_wall, existing_wall)) {
                            existing_wall = merge_walls(prev_wall, existing_wall);
                            merged = true;
                            break;
                        }
                    }
                }
                
                if (!merged) {
                    walls_to_publish.push_back(prev_wall);
                }
            }
        }
        
        auto wall_count = walls_to_publish.size();
        
        if (wall_count > 0) {
            std_msgs::msg::Float32MultiArray all_walls;
            
            all_walls.data.push_back(static_cast<float>(wall_count));
            
            for (const auto& wall : walls_to_publish) {
                for (int i = 0; i < 4; ++i) {
                    all_walls.data.push_back(wall.corners[i].x);
                    all_walls.data.push_back(wall.corners[i].y);
                    all_walls.data.push_back(wall.corners[i].z);
                }
            }
            
            walls_pub_->publish(all_walls);
            
            publishWallMarkers(walls_to_publish);
        } else {
            std_msgs::msg::Float32MultiArray empty_walls;
            empty_walls.data.push_back(0.0f);
            walls_pub_->publish(empty_walls);
            
            visualization_msgs::msg::MarkerArray marker_array;
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(delete_marker);
            marker_pub_->publish(marker_array);

            RCLCPP_INFO(this->get_logger(), "No walls detected");
        }
        
        if (rolling_window_size_ > 0) {
            if (!current_walls_.empty()) {
                if (previous_walls_.empty()) {
                    previous_walls_ = current_walls_;
                } else {
                    for (const auto& wall : current_walls_) {
                        bool merged = false;
                        if (enable_wall_merging_) {
                          for (auto& prev_wall : previous_walls_) {
                                if (should_merge_walls(wall, prev_wall)) {
                                    prev_wall = merge_walls(wall, prev_wall);
                                    merged = true;
                                    break;
                                }
                            }
                        }
                        
                        if (!merged) {
                            previous_walls_.push_back(wall);
                        }
                    }
                    
                    const size_t max_walls = static_cast<size_t>(rolling_window_size_) * 10;
                    if (previous_walls_.size() > max_walls) {
                        previous_walls_.erase(previous_walls_.begin(), 
                                             previous_walls_.begin() + (previous_walls_.size() - max_walls));
                    }
                }
            }
        } else {
            previous_walls_.clear();
        }
        
        current_walls_.clear();
        current_scan_walls_.clear();
    }

    bool should_merge_walls(const Wall& wall1, const Wall& wall2) 
    {
        float dx = wall1.center_x - wall2.center_x;
        float dy = wall1.center_y - wall2.center_y;
        float dz = wall1.center_z - wall2.center_z;
        float center_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        float max_size1 = std::max(wall1.width, wall1.height);
        float max_size2 = std::max(wall2.width, wall2.height);
        float average_size = (max_size1 + max_size2) / 2.0f;
        
        float merge_threshold = 0.3f * average_size;
        
        bool same_plane = std::abs(wall1.center_y - wall2.center_y) < 0.2f;
        
        bool x_overlap = (std::abs(wall1.center_x - wall2.center_x) < (wall1.width/2 + wall2.width/2));
        bool z_overlap = (std::abs(wall1.center_z - wall2.center_z) < (wall1.height/2 + wall2.height/2));
        
        return (center_distance < merge_threshold) || (same_plane && x_overlap && z_overlap);
    }
    
    Wall merge_walls(const Wall& wall1, const Wall& wall2) 
    {
        Wall merged;
        
        float min_x = std::min(std::min(wall1.corners[0].x, wall1.corners[2].x), 
                              std::min(wall2.corners[0].x, wall2.corners[2].x));
        float max_x = std::max(std::max(wall1.corners[0].x, wall1.corners[2].x), 
                              std::max(wall2.corners[0].x, wall2.corners[2].x));
        float min_y = std::min(std::min(wall1.corners[0].y, wall1.corners[2].y), 
                              std::min(wall2.corners[0].y, wall2.corners[2].y));
        float max_y = std::max(std::max(wall1.corners[0].y, wall1.corners[2].y), 
                              std::max(wall2.corners[0].y, wall2.corners[2].y));
        float min_z = std::min(std::min(wall1.corners[0].z, wall1.corners[2].z), 
                              std::min(wall2.corners[0].z, wall2.corners[2].z));
        float max_z = std::max(std::max(wall1.corners[0].z, wall1.corners[2].z), 
                              std::max(wall2.corners[0].z, wall2.corners[2].z));
        
        merged.center_x = (min_x + max_x) / 2.0f;
        merged.center_y = (min_y + max_y) / 2.0f;
        merged.center_z = (min_z + max_z) / 2.0f;
        merged.width = max_x - min_x;
        merged.height = max_z - min_z;
        
        merged.corners[0].x = min_x; merged.corners[0].y = min_y; merged.corners[0].z = min_z;
        merged.corners[1].x = min_x; merged.corners[1].y = min_y; merged.corners[1].z = max_z;
        merged.corners[2].x = max_x; merged.corners[2].y = max_y; merged.corners[2].z = max_z;
        merged.corners[3].x = max_x; merged.corners[3].y = max_y; merged.corners[3].z = min_z;
        
        return merged;
    }

    Eigen::Vector3f calculateWallNormal(const Wall& wall) {
        Eigen::Vector3f v1(wall.corners[1].x - wall.corners[0].x,
                          wall.corners[1].y - wall.corners[0].y,
                          wall.corners[1].z - wall.corners[0].z);
        
        Eigen::Vector3f v2(wall.corners[3].x - wall.corners[0].x,
                          wall.corners[3].y - wall.corners[0].y,
                          wall.corners[3].z - wall.corners[0].z);
        
        Eigen::Vector3f normal = v1.cross(v2);
        normal.normalize();
        
        return normal;
    }
    
    float calculatePerpendicularDistance(const Wall& wall) {
        Eigen::Vector3f normal = calculateWallNormal(wall);
        Eigen::Vector3f point_on_wall(wall.center_x, wall.center_y, wall.center_z);
        return std::abs(normal.dot(point_on_wall));
    }

    void publishWallMarkers(const std::vector<Wall>& walls) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        std::vector<Wall> smoothed_walls = applyTemporalFiltering(walls);
        
        std::unordered_set<int> active_wall_ids;
        
        for (size_t i = 0; i < smoothed_walls.size(); i++) {
            const Wall& wall = smoothed_walls[i];
            int wall_id = static_cast<int>(i);
            active_wall_ids.insert(wall_id);
            
            geometry_msgs::msg::Point p0, p1, p2, p3;
            p0.x = wall.corners[0].x; p0.y = wall.corners[0].y; p0.z = wall.corners[0].z;
            p1.x = wall.corners[1].x; p1.y = wall.corners[1].y; p1.z = wall.corners[1].z;
            p2.x = wall.corners[2].x; p2.y = wall.corners[2].y; p2.z = wall.corners[2].z;
            p3.x = wall.corners[3].x; p3.y = wall.corners[3].y; p3.z = wall.corners[3].z;
            
            visualization_msgs::msg::Marker polygon_marker;
            polygon_marker.header.frame_id = "livox_frame";
            polygon_marker.header.stamp = this->now();
            polygon_marker.ns = "wall_polygons";
            polygon_marker.id = wall_id;
            polygon_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            polygon_marker.action = visualization_msgs::msg::Marker::ADD;
            
            polygon_marker.points.push_back(p0);
            polygon_marker.points.push_back(p1);
            polygon_marker.points.push_back(p2);
            
            polygon_marker.points.push_back(p0);
            polygon_marker.points.push_back(p2);
            polygon_marker.points.push_back(p3);
            
            polygon_marker.scale.x = 1.0;
            polygon_marker.scale.y = 1.0;
            polygon_marker.scale.z = 1.0;
            polygon_marker.color.r = 1.0;
            polygon_marker.color.g = 1.0;
            polygon_marker.color.b = 0.0;
            polygon_marker.color.a = 0.5;
            
            polygon_marker.lifetime = rclcpp::Duration::from_seconds(publish_interval_ * 1.75);
            
            marker_array.markers.push_back(polygon_marker);
            
            visualization_msgs::msg::Marker outline_marker;
            outline_marker.header = polygon_marker.header;
            outline_marker.ns = "wall_outlines";
            outline_marker.id = wall_id;
            outline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            outline_marker.action = visualization_msgs::msg::Marker::ADD;
            
            outline_marker.points.push_back(p0);
            outline_marker.points.push_back(p1);
            outline_marker.points.push_back(p2);
            outline_marker.points.push_back(p3);
            outline_marker.points.push_back(p0);
            
            outline_marker.scale.x = 0.03;
            outline_marker.color.r = 1.0;
            outline_marker.color.g = 1.0;
            outline_marker.color.b = 0.0;
            outline_marker.color.a = 1.0;
            
            outline_marker.lifetime = polygon_marker.lifetime;
            
            marker_array.markers.push_back(outline_marker);
        }
        
        for (const auto& id_pair : previous_marker_ids_) {
            if (active_wall_ids.find(id_pair) == active_wall_ids.end()) {
                visualization_msgs::msg::Marker delete_marker;
                delete_marker.header.frame_id = "livox_frame";
                delete_marker.header.stamp = this->now();
                delete_marker.ns = "wall_polygons";
                delete_marker.id = id_pair;
                delete_marker.action = visualization_msgs::msg::Marker::DELETE;
                marker_array.markers.push_back(delete_marker);
                
                delete_marker.ns = "wall_outlines";
                marker_array.markers.push_back(delete_marker);
            }
        }
        
        previous_marker_ids_ = active_wall_ids;
        
        marker_pub_->publish(marker_array);
    }
    
    std::vector<Wall> applyTemporalFiltering(const std::vector<Wall>& walls) {
        std::vector<Wall> smoothed_walls = walls;
        
        if (!prev_smoothed_walls_.empty()) {
            for (auto& current_wall : smoothed_walls) {
                for (const auto& prev_wall : prev_smoothed_walls_) {
                    float dx = current_wall.center_x - prev_wall.center_x;
                    float dy = current_wall.center_y - prev_wall.center_y;
                    float dz = current_wall.center_z - prev_wall.center_z;
                    float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                    
                    if (distance < position_tolerance_) {
                        const float alpha = 0.7f;
                        
                        for (int i = 0; i < 4; ++i) {
                            current_wall.corners[i].x = alpha * current_wall.corners[i].x + 
                                                      (1.0f - alpha) * prev_wall.corners[i].x;
                            current_wall.corners[i].y = alpha * current_wall.corners[i].y + 
                                                      (1.0f - alpha) * prev_wall.corners[i].y;
                            current_wall.corners[i].z = alpha * current_wall.corners[i].z + 
                                                      (1.0f - alpha) * prev_wall.corners[i].z;
                        }
                        
                        current_wall.center_x = (current_wall.corners[0].x + current_wall.corners[2].x) / 2.0f;
                        current_wall.center_y = (current_wall.corners[0].y + current_wall.corners[2].y) / 2.0f;
                        current_wall.center_z = (current_wall.corners[0].z + current_wall.corners[2].z) / 2.0f;
                        
                        float true_width, true_height;
                        calculateTrueDimensions(current_wall, true_width, true_height);
                        current_wall.width = true_width;
                        current_wall.height = true_height;
                        
                        break;
                    }
                }
            }
        }
        
        prev_smoothed_walls_ = smoothed_walls;
        
        return smoothed_walls;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto wall_detector_node = std::make_shared<WallDetector>();
    RCLCPP_INFO(wall_detector_node->get_logger(), "Wall detector running. Interval: %.2fs", 
                wall_detector_node->get_parameter("publish_interval").as_double());
    rclcpp::spin(wall_detector_node);
    rclcpp::shutdown();
    return 0;
}