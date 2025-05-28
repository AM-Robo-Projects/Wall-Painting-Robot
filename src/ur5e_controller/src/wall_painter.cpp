#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <chrono>
#include <thread>
#include <cmath>
#include <atomic>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class ForceWallTouch : public rclcpp::Node
{
public:
    ForceWallTouch() : Node("force_wall_touch"), force_detected_(false), shutdown_requested_(false), wall_received_(false)
    {
        loadParameters();

        rclcpp::on_shutdown([this]() {
            RCLCPP_INFO(this->get_logger(), "Shutdown requested.");
            shutdown_requested_ = true;
            cleanup();
        });

        wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/force_torque_sensor_broadcaster/wrench", 10,
            std::bind(&ForceWallTouch::wrench_callback, this, std::placeholders::_1));

        wall_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "detected_walls", 10,
            std::bind(&ForceWallTouch::wall_callback, this, std::placeholders::_1));

        auto topic_list = this->get_topic_names_and_types();
        bool force_topic_found = false;
        for (const auto &topic : topic_list)
        {
            if (topic.first == "/force_torque_sensor_broadcaster/wrench")
            {
                force_topic_found = true;
                break;
            }
        }
        
        if (!force_topic_found)
        {
            RCLCPP_ERROR(this->get_logger(), "Force topic NOT found! Is the force torque sensor publisher running?");
        }
    }

    void run()
    {
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);

        move_group_node_ = std::make_shared<rclcpp::Node>(
            "force_wall_touch_moveit_node", node_options);

        configureMoveItLoggerLevels();

        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(move_group_node_);
        executor_->add_node(shared_from_this());

        spinner_ = std::make_shared<std::thread>([this]() {
            while (rclcpp::ok() && !shutdown_requested_) {
                if (executor_) executor_->spin_some(std::chrono::milliseconds(100));
            }
        });

        std::this_thread::sleep_for(std::chrono::seconds(2));

        if (shutdown_requested_)
        {
            RCLCPP_INFO(this->get_logger(), "Shutdown requested during startup");
            cleanup();
            return;
        }

        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node_, "ur_manipulator");
        move_group_->setMaxVelocityScalingFactor(0.1);
        move_group_->setMaxAccelerationScalingFactor(0.1);
        move_group_->setPlanningTime(1.0);

        moveToHome();
        rclcpp::sleep_for(std::chrono::milliseconds(1500));

        int wait_count = 0;
        while (!wall_received_ && rclcpp::ok() && wait_count < 30)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            wait_count++;
        }
        
        if (!wall_received_)
        {
            RCLCPP_ERROR(this->get_logger(), "No wall detected from wall_detector! Aborting.");
            moveToHome();
            return;
        }

        if (wall_corners_.size() < 12)
        {
            RCLCPP_ERROR(this->get_logger(), "Wall data received but not enough corners (need 4).");
            moveToHome();
            return;
        }
        
        geometry_msgs::msg::Point top1, top2;
        top1.x = wall_corners_[0];
        top1.y = wall_corners_[1];
        top1.z = wall_corners_[2];
        top2.x = wall_corners_[3];
        top2.y = wall_corners_[4];
        top2.z = wall_corners_[5];

        double min_z = wall_corners_[2];
        for (size_t i = 2; i < wall_corners_.size(); i += 3)
        {
            if (wall_corners_[i] < min_z)
                min_z = wall_corners_[i];
        }

        paint_wall(top1, top2, min_z);
    }

    void cleanup()
    {
        if (executor_)
        {
            executor_->cancel();
        }
        if (spinner_ && spinner_->joinable())
        {
            spinner_->join();
        }
    }

    ~ForceWallTouch()
    {
        cleanup();
    }

private:
    std::shared_ptr<rclcpp::Node> move_group_node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::shared_ptr<std::thread> spinner_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wall_sub_;
    std::atomic<bool> force_detected_;

    geometry_msgs::msg::WrenchStamped latest_wrench_;
    std::mutex wrench_mutex_;

    double horizontal_step_;
    double approach_step_;
    int max_approach_steps_;
    double velocity_scale_;
    double force_threshold_;
    double tool_size_;
    int wall_direction_;
    bool approach_in_x_;
    double approach_distance_x_;
    double approach_distance_y_;
    int num_refuels_;

    bool shutdown_requested_;

    std::vector<float> wall_corners_;
    std::mutex wall_mutex_;
    std::atomic<bool> wall_received_;

    double home_x_, home_y_, home_z_;
    double home_roll_, home_pitch_, home_yaw_;

    double painting_orientation_roll_, painting_orientation_pitch_, painting_orientation_yaw_;

    std::vector<geometry_msgs::msg::Point> top_contact_points_;
    std::vector<geometry_msgs::msg::Point> bottom_contact_points_;

    class ProgressBar
    {
    public:
        ProgressBar(size_t total, size_t width = 50,
                    std::string prefix = "Progress: ",
                    std::string fill = "█",
                    std::string empty = "░")
            : total_(total), width_(width), prefix_(prefix),
              fill_char_(fill), empty_char_(empty),
              last_printed_length_(0), position_(0) {}

        void update(size_t position)
        {
            position_ = position;
            float progress = static_cast<float>(position) / static_cast<float>(total_);
            std::cout << '\r';
            std::cout << prefix_;
            size_t filled_width = static_cast<size_t>(width_ * progress);
            std::string bar;
            for (size_t i = 0; i < width_; ++i)
            {
                if (i < filled_width)
                {
                    bar += fill_char_;
                }
                else
                {
                    bar += empty_char_;
                }
            }
            int percent = static_cast<int>(100 * progress);
            std::string output = bar + " " + std::to_string(percent) + "% (" +
                                std::to_string(position) + "/" + std::to_string(total_) + ")";
            std::cout << output;
            last_printed_length_ = output.length() + prefix_.length();
            std::cout << std::flush;
        }

        void finish()
        {
            update(total_);
            std::cout << std::endl;
        }

    private:
        size_t total_;
        size_t width_;
        std::string prefix_;
        std::string fill_char_;
        std::string empty_char_;
        size_t last_printed_length_;
        size_t position_;
    };

    void configureMoveItLoggerLevels()
    {
        const std::vector<std::string> loggers_to_silence = {
            "move_group_interface",
            "planning_scene_monitor",
            "planning_pipeline",
            "robot_model_loader",
            "kinematics_plugin_loader",
            "robot_state",
            "planning_interface",
            "moveit_ros.current_state_monitor"};

        for (const auto &logger_name : loggers_to_silence)
        {
            auto ret = rcutils_logging_set_logger_level(
                logger_name.c_str(), RCUTILS_LOG_SEVERITY_WARN);
            if (ret != RCUTILS_RET_OK)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to set logger level for %s", logger_name.c_str());
            }
        }
    }

    geometry_msgs::msg::WrenchStamped get_latest_wrench()
    {
        static geometry_msgs::msg::WrenchStamped latest_wrench;
        static std::mutex wrench_mutex;

        {
            std::lock_guard<std::mutex> lock(wrench_mutex);
            return latest_wrench_;
        }
    }

    void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        if (std::abs(msg->wrench.force.z) > force_threshold_)
        {
            force_detected_ = true;
        }

        std::lock_guard<std::mutex> lock(wrench_mutex_);
        latest_wrench_ = *msg;
    }

    bool force_feedback_detected()
    {
        return force_detected_;
    }

    void loadParameters()
    {
        try
        {
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("ur5e_controller");
            std::string config_file = package_share_directory + "/config/painting_config.yaml";

            YAML::Node config = YAML::LoadFile(config_file);

            if (!config["painting"])
            {
                RCLCPP_ERROR(this->get_logger(), "Missing 'painting' section in config file");
                rclcpp::shutdown();
                exit(EXIT_FAILURE);
            }

            auto painting_config = config["painting"];
            if (!painting_config["horizontal_step"] || !painting_config["force_threshold"])
            {
                RCLCPP_ERROR(this->get_logger(), "Missing required painting parameters (horizontal_step, force_threshold)");
                rclcpp::shutdown();
                exit(EXIT_FAILURE);
            }

            horizontal_step_ = painting_config["horizontal_step"].as<double>();
            force_threshold_ = painting_config["force_threshold"].as<double>();
            
            if (painting_config["num_refuels"])
            {
                num_refuels_ = painting_config["num_refuels"].as<int>();
            }
            else
            {
                num_refuels_ = 0;
            }

            // Load painting orientation (default to M_PI, M_PI/2, 0.0 if not present)
            if (painting_config["painting_orientation_roll"] && 
                painting_config["painting_orientation_pitch"] && 
                painting_config["painting_orientation_yaw"])
            {
                painting_orientation_roll_ = painting_config["painting_orientation_roll"].as<double>();
                painting_orientation_pitch_ = painting_config["painting_orientation_pitch"].as<double>();
                painting_orientation_yaw_ = painting_config["painting_orientation_yaw"].as<double>();
            }
            else
            {
                painting_orientation_roll_ = M_PI;
                painting_orientation_pitch_ = M_PI / 2;
                painting_orientation_yaw_ = 0.0;
            }

            if (!config["motion"])
            {
                RCLCPP_ERROR(this->get_logger(), "Missing 'motion' section in config file");
                rclcpp::shutdown();
                exit(EXIT_FAILURE);
            }

            auto motion_config = config["motion"];
            if (!motion_config["approach_step"] || !motion_config["max_approach_steps"] ||
                !motion_config["velocity_scale"])
            {
                RCLCPP_ERROR(this->get_logger(), "Missing required motion parameters");
                rclcpp::shutdown();
                exit(EXIT_FAILURE);
            }

            approach_step_ = motion_config["approach_step"].as<double>();
            max_approach_steps_ = motion_config["max_approach_steps"].as<int>();
            velocity_scale_ = motion_config["velocity_scale"].as<double>();
            
            if (motion_config["tool_size"])
            {
                tool_size_ = motion_config["tool_size"].as<double>();
            }
            else
            {
                tool_size_ = 0.05;
            }

            if (!config["home_position"])
            {
                RCLCPP_ERROR(this->get_logger(), "Missing 'home_position' section in config file");
                rclcpp::shutdown();
                exit(EXIT_FAILURE);
            }
            
            auto home_config = config["home_position"];
            if (!home_config["x"] || !home_config["y"] || !home_config["z"] ||
                !home_config["roll"] || !home_config["pitch"] || !home_config["yaw"])
            {
                RCLCPP_ERROR(this->get_logger(), "Missing required home_position parameters (x, y, z, roll, pitch, yaw)");
                rclcpp::shutdown();
                exit(EXIT_FAILURE);
            }
            
            home_x_ = home_config["x"].as<double>();
            home_y_ = home_config["y"].as<double>();
            home_z_ = home_config["z"].as<double>();
            home_roll_ = home_config["roll"].as<double>();
            home_pitch_ = home_config["pitch"].as<double>();
            home_yaw_ = home_config["yaw"].as<double>();
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
            rclcpp::shutdown();
            exit(EXIT_FAILURE);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load config file: %s", e.what());
            rclcpp::shutdown();
            exit(EXIT_FAILURE);
        }
    }

    void paint_wall(const geometry_msgs::msg::Point &top1, const geometry_msgs::msg::Point &top2, double min_z)
    {
        approach_in_x_ = std::abs(top1.x) > std::abs(top1.y);
        wall_direction_ = approach_in_x_ ? sgn(top1.x) : sgn(top1.y);

        approach_distance_x_ = approach_in_x_ ? approach_step_ * wall_direction_ : 0.0;
        approach_distance_y_ = approach_in_x_ ? 0.0 : approach_step_ * wall_direction_;

        geometry_msgs::msg::Point offset_top1 = top1;
        geometry_msgs::msg::Point offset_top2 = top2;
        if (approach_in_x_)
        {
            offset_top1.x -= tool_size_ * wall_direction_;
            offset_top2.x -= tool_size_ * wall_direction_;
        }
        else
        {
            offset_top1.y -= tool_size_ * wall_direction_;
            offset_top2.y -= tool_size_ * wall_direction_;
        }

        double total_distance = std::sqrt(std::pow(offset_top2.x - offset_top1.x, 2) + 
                                         std::pow(offset_top2.y - offset_top1.y, 2));
        int num_columns = static_cast<int>(total_distance / horizontal_step_) + 1;

        top_contact_points_.clear();
        bottom_contact_points_.clear();

        ProgressBar progress_bar(num_columns + 1, 40, "Painting Progress: ");

        bool direction_down = true;

        std::vector<int> refuel_columns;
        if (num_refuels_ > 0 && num_columns > 0)
        {
            for (int r = 1; r <= num_refuels_; ++r)
            {
                int col = static_cast<int>(std::round(r * num_columns / double(num_refuels_ + 1)));
                refuel_columns.push_back(col);
            }
        }

        for (int col = 0; col <= num_columns; col++)
        {
            if (shutdown_requested_ || !rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "Shutdown requested, stopping painting.");
                progress_bar.finish();
                moveToHome();
                return;
            }

            progress_bar.update(col);

            double t = (num_columns > 0) ? static_cast<double>(col) / num_columns : 0.0;

            geometry_msgs::msg::Point current_top, current_bottom;
            current_top.x = offset_top1.x + t * (offset_top2.x - offset_top1.x);
            current_top.y = offset_top1.y + t * (offset_top2.y - offset_top1.y);
            current_top.z = offset_top1.z;

            current_bottom.x = current_top.x;
            current_bottom.y = current_top.y;
            current_bottom.z = min_z;

            geometry_msgs::msg::Point start_point = direction_down ? current_top : current_bottom;
            geometry_msgs::msg::Point end_z_coordinate = direction_down ? current_bottom : current_top;

            moveToPoint(start_point);

            geometry_msgs::msg::Point contact_point;
            if (!approachWallUntilContact(start_point, contact_point))
            {
                moveToHome();
                progress_bar.finish();
                return;
            }

            geometry_msgs::msg::Point top_contact = contact_point;
            top_contact.z = current_top.z;
            geometry_msgs::msg::Point bottom_contact = contact_point;
            bottom_contact.z = current_bottom.z;

            top_contact_points_.push_back(top_contact);
            bottom_contact_points_.push_back(bottom_contact);

            contact_point.z = end_z_coordinate.z;
            moveToPoint(contact_point, velocity_scale_);

            direction_down = !direction_down;

            if (std::find(refuel_columns.begin(), refuel_columns.end(), col) != refuel_columns.end())
            {
                if (num_refuels_ > 0)
                {
                    performRefuel();
                }
            }

            if (shutdown_requested_ || !rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "Shutdown requested, stopping painting.");
                progress_bar.finish();
                break;
            }
        }

        progress_bar.finish();

        if (!top_contact_points_.empty() && !bottom_contact_points_.empty())
        {
            performRoundingPass();
        }

        RCLCPP_INFO(this->get_logger(), "Wall painting completed!");
        moveToHome();
        rclcpp::shutdown();
    }

    void performRoundingPass()
    {
        if (top_contact_points_.empty() || bottom_contact_points_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No contact points saved, skipping rounding pass");
            return;
        }

        const double contact_point_scale = 1;

        auto scale_points = [&](const std::vector<geometry_msgs::msg::Point> &points)
        {
            std::vector<geometry_msgs::msg::Point> scaled_points;
            for (const auto &point : points)
            {
                geometry_msgs::msg::Point scaled_point = point;
                if (approach_in_x_)
                {
                    scaled_point.x *= contact_point_scale;
                }
                else
                {
                    scaled_point.y *= contact_point_scale;
                }
                scaled_points.push_back(scaled_point);
            }
            return scaled_points;
        };

        auto rotateZ90Degrees = [](const geometry_msgs::msg::Pose &input_pose) -> geometry_msgs::msg::Pose
        {
            // Convert input pose to Eigen transformation
            Eigen::Isometry3d pose_eigen;
            tf2::fromMsg(input_pose, pose_eigen);

            // Apply 90-degree (M_PI/2) rotation around Z-axis in tool frame
            // Note: multiplying on the right applies the rotation in the tool frame
            pose_eigen = pose_eigen * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());

            // Convert back to geometry_msgs::Pose and return
            geometry_msgs::msg::Pose rotated_pose;
            rotated_pose = tf2::toMsg(pose_eigen);
            return rotated_pose;
        };

        std::vector<geometry_msgs::msg::Point> scaled_top_points = scale_points(top_contact_points_);
        std::vector<geometry_msgs::msg::Point> scaled_bottom_points = scale_points(bottom_contact_points_);

        std::vector<geometry_msgs::msg::Point> simplified_top_points = simplifyPoints(scaled_top_points, 5);
        std::vector<geometry_msgs::msg::Point> simplified_bottom_points = simplifyPoints(scaled_bottom_points, 5);

        RCLCPP_INFO(this->get_logger(), "Starting rounding pass with simplified waypoints (%zu top, %zu bottom)...",
                    simplified_top_points.size(), simplified_bottom_points.size());

        std::vector<geometry_msgs::msg::Pose> waypoints;

        // Current position to starting approach
        geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
        waypoints.push_back(current_pose.pose);

        // Slightly scaled back position for safety
        geometry_msgs::msg::PoseStamped current_pose_scaled = move_group_->getCurrentPose();
        current_pose_scaled.pose.position.x *= .95;
        current_pose_scaled.pose.position.y *= .95;
        waypoints.push_back(current_pose_scaled.pose);

        // Approach to first point
        geometry_msgs::msg::Pose first_pose;
        first_pose.position = simplified_top_points.front();
        first_pose.position.x *= 0.95;
        first_pose.position.y *= 0.95;
        tf2::Quaternion q;
        q.setRPY(painting_orientation_roll_, painting_orientation_pitch_, painting_orientation_yaw_);
        first_pose.orientation = tf2::toMsg(q);
        waypoints.push_back(first_pose);

        // Add rotated waypoint at same position before starting horizontal movement
        geometry_msgs::msg::Pose horizontal_first_pose = rotateZ90Degrees(first_pose);
        waypoints.push_back(horizontal_first_pose);

        // Top edge - horizontal movement (rotated 90 degrees)
        for (size_t i = 0; i < simplified_top_points.size(); ++i)
        {
            geometry_msgs::msg::Pose pose;
            pose.position = simplified_top_points[i];
            q.setRPY(painting_orientation_roll_, painting_orientation_pitch_, painting_orientation_yaw_);
            pose.orientation = tf2::toMsg(q);
            // Apply 90 degree rotation for horizontal movement
            pose.position.z *= 1.02;
            waypoints.push_back(rotateZ90Degrees(pose));
        }

        if (!simplified_bottom_points.empty())
        {
            // Last point on top horizontal path
            geometry_msgs::msg::Pose last_top_pose;
            last_top_pose.position = simplified_top_points.back();
            q.setRPY(painting_orientation_roll_, painting_orientation_pitch_, painting_orientation_yaw_);
            last_top_pose.orientation = tf2::toMsg(q);
            
            // Add unrotated waypoint for transition to vertical movement
            waypoints.push_back(last_top_pose);
            
            // Vertical movement down to bottom (unrotated)
            geometry_msgs::msg::Pose bottom_pose;
            bottom_pose.position = simplified_bottom_points.back();
            q.setRPY(painting_orientation_roll_, painting_orientation_pitch_, painting_orientation_yaw_);
            bottom_pose.orientation = tf2::toMsg(q);
            waypoints.push_back(bottom_pose);
            
            // Add rotated waypoint at same position before starting horizontal movement on bottom
            waypoints.push_back(rotateZ90Degrees(bottom_pose));
        }

        // Bottom edge - horizontal movement (rotated 90 degrees)
        for (int i = static_cast<int>(simplified_bottom_points.size()) - 2; i >= 0; --i)
        {
            geometry_msgs::msg::Pose pose;
            pose.position = simplified_bottom_points[i];
            q.setRPY(painting_orientation_roll_, painting_orientation_pitch_, painting_orientation_yaw_);
            pose.orientation = tf2::toMsg(q);
            // Apply 90 degree rotation for horizontal movement
            waypoints.push_back(rotateZ90Degrees(pose));
        }

        if (!simplified_top_points.empty())
        {
            // First point on bottom horizontal path
            geometry_msgs::msg::Pose first_bottom_pose;
            first_bottom_pose.position = simplified_bottom_points.front();
            q.setRPY(painting_orientation_roll_, painting_orientation_pitch_, painting_orientation_yaw_);
            first_bottom_pose.orientation = tf2::toMsg(q);
            
            // Add unrotated waypoint for transition to vertical movement
            waypoints.push_back(first_bottom_pose);
            
            // Vertical movement up to top (unrotated)
            geometry_msgs::msg::Pose top_pose;
            top_pose.position = simplified_top_points.front();
            q.setRPY(painting_orientation_roll_, painting_orientation_pitch_, painting_orientation_yaw_);
            top_pose.orientation = tf2::toMsg(q);
            waypoints.push_back(top_pose);
        }

        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(velocity_scale_);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.005;
        const double jump_threshold = 0.0;

        double fraction = move_group_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

        if (fraction > 0.95)
        {
            RCLCPP_INFO(this->get_logger(), "Executing smooth rounding pass (%.2f%% of path computed)", fraction * 100);
            move_group_->execute(trajectory);
            RCLCPP_INFO(this->get_logger(), "Smooth horizontal rounding pass completed!");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Could not compute complete rounding path (only %.2f%% computed), skipping", 
                        fraction * 100);
        }
    }

    std::vector<geometry_msgs::msg::Point> simplifyPoints(
        const std::vector<geometry_msgs::msg::Point> &points, size_t target_count)
    {
        if (points.size() <= target_count)
        {
            return points;
        }

        std::vector<geometry_msgs::msg::Point> result;

        result.push_back(points.front());

        if (target_count > 2)
        {
            size_t internal_points = target_count - 2;
            double segment = static_cast<double>(points.size() - 2) / internal_points;
            for (size_t i = 1; i <= internal_points; ++i)
            {
                size_t index = 1 + static_cast<size_t>(i * segment);
                if (index >= points.size() - 1)
                {
                    index = points.size() - 2;
                }
                result.push_back(points[index]);
            }
        }

        result.push_back(points.back());

        return result;
    }

    void performRefuel()
    {
        moveToHome();
        geometry_msgs::msg::PoseStamped home_pose = move_group_->getCurrentPose();
        geometry_msgs::msg::Pose down_pose = home_pose.pose;
        down_pose.position.z -= 0.05;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(home_pose.pose);
        waypoints.push_back(down_pose);

        move_group_->setMaxVelocityScalingFactor(0.1);
        move_group_->setMaxAccelerationScalingFactor(0.1);
        move_group_->setPlanningTime(1.0);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.005;
        const double jump_threshold = 0.0;

        double fraction = move_group_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

        if (fraction > 0.95)
        {
            move_group_->execute(trajectory);
        }

        waypoints.clear();
        waypoints.push_back(down_pose);
        waypoints.push_back(home_pose.pose);
        fraction = move_group_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);
        if (fraction > 0.95)
        {
            move_group_->execute(trajectory);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        waypoints.clear();
        waypoints.push_back(home_pose.pose);
        waypoints.push_back(down_pose);
        fraction = move_group_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);
        if (fraction > 0.95)
        {
            move_group_->execute(trajectory);
        }

        waypoints.clear();
        waypoints.push_back(down_pose);
        waypoints.push_back(home_pose.pose);
        fraction = move_group_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);
        if (fraction > 0.95)
        {
            move_group_->execute(trajectory);
        }
    }

    bool approachWallUntilContact(const geometry_msgs::msg::Point &start_point, 
                                 geometry_msgs::msg::Point &contact_point)
    {
        geometry_msgs::msg::Point approach = start_point;
        force_detected_ = false;
        contact_point = start_point;

        for (int i = 0; i < max_approach_steps_; ++i)
        {
            if (shutdown_requested_ || !rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "Shutdown requested during wall approach, aborting.");
                return false;
            }

            if (force_feedback_detected())
            {
                geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
                contact_point = current_pose.pose.position;
                force_detected_ = false;
                return true;
            }

            approach.x += approach_distance_x_;
            approach.y += approach_distance_y_;

            moveToPoint(approach, velocity_scale_);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        RCLCPP_ERROR(this->get_logger(), "Could not establish wall contact after %d steps", max_approach_steps_);
        rclcpp::shutdown();
        return false;
    }

    void moveToHome()
    {
        geometry_msgs::msg::Pose home_pose;
        home_pose.position.x = home_x_;
        home_pose.position.y = home_y_;
        home_pose.position.z = home_z_;
        tf2::Quaternion q;
        q.setRPY(home_roll_, home_pitch_, home_yaw_);
        home_pose.orientation = tf2::toMsg(q);

        geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(current_pose.pose);
        waypoints.push_back(home_pose);

        move_group_->setMaxVelocityScalingFactor(0.1);
        move_group_->setMaxAccelerationScalingFactor(0.1);
        move_group_->setPlanningTime(1.0);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.005;
        const double jump_threshold = 0.0;

        double fraction = move_group_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

        if (fraction > 0.95)
        {
            move_group_->execute(trajectory);
        }
    }

    void moveToPoint(const geometry_msgs::msg::Point &target_point, double vel_scale = 0.05)
    {
        geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
        geometry_msgs::msg::Pose target_pose;
        target_pose.position = target_point;
        tf2::Quaternion q;
        q.setRPY(painting_orientation_roll_, painting_orientation_pitch_, painting_orientation_yaw_);
        target_pose.orientation = tf2::toMsg(q);

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(current_pose.pose);
        waypoints.push_back(target_pose);

        move_group_->setMaxVelocityScalingFactor(vel_scale);
        move_group_->setMaxAccelerationScalingFactor(vel_scale);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.005;
        const double jump_threshold = 0.0;

        double fraction = move_group_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

        if (fraction > 0.95)
        {
            move_group_->execute(trajectory);
        }
    }

    void wall_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 12)
        {
            std::lock_guard<std::mutex> lock(wall_mutex_);
            wall_corners_ = msg->data;
            wall_received_ = true;
        }
    }

    static int sgn(double val)
    {
        return (val > 0) - (val < 0);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ForceWallTouch>();

    rclcpp::spin_some(node);

    node->run();

    node->cleanup();

    rclcpp::shutdown();
    return 0;
}
