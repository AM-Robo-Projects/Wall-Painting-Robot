#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <atomic>

std::atomic<bool> g_shutdown_requested(false);

void signalHandler(int signum) {
    (void)signum;
    g_shutdown_requested = true;
    std::cout << "\nShutdown requested. Exiting...\n";
}

struct Wall {
    std::vector<geometry_msgs::msg::Point> corners;
};

class WallPainter : public rclcpp::Node {
public:
    WallPainter() : Node("wall_painter"), initial_setup_(false) {
        signal(SIGINT, signalHandler);
        configureTerminal();
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        move_group_node_ = std::make_shared<rclcpp::Node>(
            "wall_painter_moveit_node", 
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );
        executor_.add_node(move_group_node_);
        spinner_ = std::make_shared<std::thread>([this]() { this->executor_.spin(); });
        RCLCPP_INFO(this->get_logger(), "Setting up MoveIt interface...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node_, "ur_manipulator");
        move_group_->setMaxVelocityScalingFactor(0.1);
        move_group_->setMaxAccelerationScalingFactor(0.1);
        move_group_->setPlanningTime(1.0);
        initial_setup_ = true;
        RCLCPP_INFO(this->get_logger(), "Moving to home position...");
        moveToHome();
        wall_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "detected_walls", 10, std::bind(&WallPainter::wall_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Ready to detect walls.");
        shutdown_thread_ = std::make_shared<std::thread>([this]() {
            while (!g_shutdown_requested && rclcpp::ok()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (g_shutdown_requested) {
                RCLCPP_INFO(this->get_logger(), "Shutdown requested, terminating...");
                rclcpp::shutdown();
            }
        });
    }
    
    ~WallPainter() {
        g_shutdown_requested = true;
        if (shutdown_thread_ && shutdown_thread_->joinable()) {
            shutdown_thread_->join();
        }
        if (spinner_) {
            executor_.cancel();
            if (spinner_->joinable()) {
                spinner_->join();
            }
        }
        restoreTerminal();
    }

    void restoreTerminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
    }

private:
    void moveToHome() {
        RCLCPP_INFO(this->get_logger(), "Moving to home position (X=0.0027, Y=0.3267, Z=0.3998)...");
        geometry_msgs::msg::Pose home_pose;
        home_pose.position.x = 0.0027;
        home_pose.position.y = 0.3267;
        home_pose.position.z = 0.3998;
        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, 0.0);
        home_pose.orientation = tf2::toMsg(q);
        geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(current_pose.pose);
        waypoints.push_back(home_pose);
        move_group_->setMaxVelocityScalingFactor(0.1);
        move_group_->setMaxAccelerationScalingFactor(0.1);
        move_group_->setPlanningTime(1.0);
        RCLCPP_INFO(this->get_logger(), "Planning Cartesian path to home...");
        // --- Add missing variable declarations ---
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.005;
        const double jump_threshold = 0.0;
        double fraction = move_group_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);
        if (fraction > 0.95) {
            RCLCPP_INFO(this->get_logger(), "Cartesian path to home computed (%.2f%% achieved)", fraction * 100.0);
            bool cart_success = (move_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
            if (cart_success) {
                RCLCPP_INFO(this->get_logger(), "Home position reached successfully");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to execute Cartesian path to home");
                restoreTerminal();
                rclcpp::shutdown();
                exit(1);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute Cartesian path to home (only %.2f%% achieved)", fraction * 100.0);
            restoreTerminal();
            rclcpp::shutdown();
            exit(1);
        }
    }
    
    void wall_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (g_shutdown_requested) {
            return;
        }
        if (msg->data.size() != 12) {
            RCLCPP_WARN(this->get_logger(), "Expected 12 floats (4 corners), got %zu. Ignoring.", msg->data.size());
            return;
        }
        Wall wall;
        for (int i = 0; i < 4; ++i) {
            geometry_msgs::msg::Point corner;
            corner.x = msg->data[i * 3 + 0];
            corner.y = msg->data[i * 3 + 1];
            corner.z = msg->data[i * 3 + 2];
            wall.corners.push_back(corner);
        }
        processWall(wall);
        wall_subscription_.reset();
    }
    
    void processWall(const Wall& wall) {
        RCLCPP_INFO(this->get_logger(), "=== Wall Found ===");
        RCLCPP_INFO(this->get_logger(), "Corner coordinates in LiDAR frame:");
        for (size_t i = 0; i < wall.corners.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Corner %zu: (%.3f, %.3f, %.3f)",
                       i, wall.corners[i].x, wall.corners[i].y, wall.corners[i].z);
        }

        // --- Find the two top corners (highest z), assign left/right by x ---
        std::vector<size_t> top_indices;
        float max_z = -std::numeric_limits<float>::infinity();
        for (size_t i = 0; i < wall.corners.size(); ++i) {
            if (wall.corners[i].z > max_z) {
                max_z = wall.corners[i].z;
            }
        }
        for (size_t i = 0; i < wall.corners.size(); ++i) {
            if (std::abs(wall.corners[i].z - max_z) < 1e-4) {
                top_indices.push_back(i);
            }
        }
        while (top_indices.size() > 2) {
            size_t min_y_idx = top_indices[0];
            for (size_t i = 1; i < top_indices.size(); ++i) {
                if (wall.corners[top_indices[i]].y < wall.corners[min_y_idx].y) {
                    min_y_idx = top_indices[i];
                }
            }
            top_indices.erase(std::remove(top_indices.begin(), top_indices.end(), min_y_idx), top_indices.end());
        }
        size_t top_left_idx, top_right_idx;
        if (wall.corners[top_indices[0]].x < wall.corners[top_indices[1]].x) {
            top_left_idx = top_indices[0];
            top_right_idx = top_indices[1];
        } else {
            top_left_idx = top_indices[1];
            top_right_idx = top_indices[0];
        }
        RCLCPP_INFO(this->get_logger(), "Selected top corners: idx %zu (z=%.3f, x=%.3f) and idx %zu (z=%.3f, x=%.3f)",
            top_left_idx, wall.corners[top_left_idx].z, wall.corners[top_left_idx].x,
            top_right_idx, wall.corners[top_right_idx].z, wall.corners[top_right_idx].x);

        // --- Find the two bottom corners (lowest z), assign left/right by x ---
        std::vector<size_t> bottom_indices;
        float min_z = std::numeric_limits<float>::infinity();
        for (size_t i = 0; i < wall.corners.size(); ++i) {
            if (wall.corners[i].z < min_z) {
                min_z = wall.corners[i].z;
            }
        }
        for (size_t i = 0; i < wall.corners.size(); ++i) {
            if (std::abs(wall.corners[i].z - min_z) < 1e-4) {
                bottom_indices.push_back(i);
            }
        }
        while (bottom_indices.size() > 2) {
            size_t min_y_idx = bottom_indices[0];
            for (size_t i = 1; i < bottom_indices.size(); ++i) {
                if (wall.corners[bottom_indices[i]].y < wall.corners[min_y_idx].y) {
                    min_y_idx = bottom_indices[i];
                }
            }
            bottom_indices.erase(std::remove(bottom_indices.begin(), bottom_indices.end(), min_y_idx), bottom_indices.end());
        }
        size_t bottom_left_idx, bottom_right_idx;
        if (wall.corners[bottom_indices[0]].x < wall.corners[bottom_indices[1]].x) {
            bottom_left_idx = bottom_indices[0];
            bottom_right_idx = bottom_indices[1];
        } else {
            bottom_left_idx = bottom_indices[1];
            bottom_right_idx = bottom_indices[0];
        }
        RCLCPP_INFO(this->get_logger(), "Selected bottom corners: idx %zu (z=%.3f, x=%.3f) and idx %zu (z=%.3f, x=%.3f)",
            bottom_left_idx, wall.corners[bottom_left_idx].z, wall.corners[bottom_left_idx].x,
            bottom_right_idx, wall.corners[bottom_right_idx].z, wall.corners[bottom_right_idx].x);

        // Only transform the two top corners (no normal/offset logic)
        auto approach_top_left = transformPoint(wall.corners[top_left_idx]);
        auto approach_top_right = transformPoint(wall.corners[top_right_idx]);
        auto approach_bottom_left = transformPoint(wall.corners[bottom_left_idx]);
        auto approach_bottom_right = transformPoint(wall.corners[bottom_right_idx]);

        // --- Scale x and y toward zero to bring points closer to robot base ---
        const double scale_xy = 0.8; // 0 < scale_xy < 1, smaller = closer to (0,0)
        approach_top_left.x *= scale_xy;
        approach_top_left.y *= scale_xy;
        approach_top_right.x *= scale_xy;
        approach_top_right.y *= scale_xy;
        approach_bottom_left.x *= scale_xy;
        approach_bottom_left.y *= scale_xy;
        approach_bottom_right.x *= scale_xy;
        approach_bottom_right.y *= scale_xy;

        // Cap the z value at 0.55 for top, min at 0.15 for bottom
        if (approach_top_left.z > 0.55) approach_top_left.z = 0.55;
        if (approach_top_right.z > 0.55) approach_top_right.z = 0.55;
        if (approach_bottom_left.z < 0.15) approach_bottom_left.z = 0.15;
        if (approach_bottom_right.z < 0.15) approach_bottom_right.z = 0.15;

        // --- LOGGING AND MOVEMENT ---
        RCLCPP_INFO(this->get_logger(), "Approach Top Left: (%.3f, %.3f, %.3f)", 
                   approach_top_left.x, approach_top_left.y, approach_top_left.z);
        RCLCPP_INFO(this->get_logger(), "Approach Top Right: (%.3f, %.3f, %.3f)", 
                   approach_top_right.x, approach_top_right.y, approach_top_right.z);
        RCLCPP_INFO(this->get_logger(), "Approach Bottom Left: (%.3f, %.3f, %.3f)", 
                   approach_bottom_left.x, approach_bottom_left.y, approach_bottom_left.z);
        RCLCPP_INFO(this->get_logger(), "Approach Bottom Right: (%.3f, %.3f, %.3f)", 
                   approach_bottom_right.x, approach_bottom_right.y, approach_bottom_right.z);

        RCLCPP_INFO(this->get_logger(), "Top corner coordinates in robot base_link frame (adjusted):");
        RCLCPP_INFO(this->get_logger(), "Top Left: (%.3f, %.3f, %.3f)", 
                   approach_top_left.x, approach_top_left.y, approach_top_left.z);
        RCLCPP_INFO(this->get_logger(), "Top Right: (%.3f, %.3f, %.3f)", 
                   approach_top_right.x, approach_top_right.y, approach_top_right.z);
        RCLCPP_INFO(this->get_logger(), "Bottom Left: (%.3f, %.3f, %.3f)", 
                   approach_bottom_left.x, approach_bottom_left.y, approach_bottom_left.z);
        RCLCPP_INFO(this->get_logger(), "Bottom Right: (%.3f, %.3f, %.3f)", 
                   approach_bottom_right.x, approach_bottom_right.y, approach_bottom_right.z);

        moveToPoint(approach_top_left);
        moveToPoint(approach_top_right);
        moveToPoint(approach_bottom_right);
        moveToPoint(approach_bottom_left);
        RCLCPP_INFO(this->get_logger(), "Movement completed. Exiting...");
        restoreTerminal();
        rclcpp::shutdown();
    }
    
    geometry_msgs::msg::Point transformPoint(const geometry_msgs::msg::Point& point_in_lidar) {
        // Use tf2_ros to transform the point from LiDAR frame to robot base frame
        geometry_msgs::msg::PointStamped input_point, output_point;
        input_point.header.frame_id = "livox_frame"; // must match static_transform_publisher child_frame
        input_point.header.stamp = this->now();
        input_point.point = point_in_lidar;
        try {
            // Wait for transform to be available
            if (!tf_buffer_->canTransform("base", "livox_frame", tf2::TimePointZero, tf2::durationFromSec(1.0))) {
                RCLCPP_ERROR(this->get_logger(), "Transform from livox_frame to base not available!");
                restoreTerminal();
                rclcpp::shutdown();
                exit(1);
            }
            output_point = tf_buffer_->transform(input_point, "base", tf2::durationFromSec(1.0));
            // change the output point to -x and -y
            output_point.point.x = -output_point.point.x;
            output_point.point.y = -output_point.point.y;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "TF transform failed: %s", ex.what());
            restoreTerminal();
            rclcpp::shutdown();
            exit(1);
        }
        RCLCPP_INFO(this->get_logger(), "Point transformed (tf2) from (%.3f, %.3f, %.3f) to (%.3f, %.3f, %.3f)",
            point_in_lidar.x, point_in_lidar.y, point_in_lidar.z,
            output_point.point.x, output_point.point.y, output_point.point.z);
        return output_point.point;
    }
    
    void moveToPoint(const geometry_msgs::msg::Point& target_point) {
        RCLCPP_INFO(this->get_logger(), "Moving to position: (%.3f, %.3f, %.3f)",
                   target_point.x, target_point.y, target_point.z);
        geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
        RCLCPP_INFO(this->get_logger(), "Current position: (%.3f, %.3f, %.3f)",
                   current_pose.pose.position.x, 
                   current_pose.pose.position.y, 
                   current_pose.pose.position.z);
        geometry_msgs::msg::Pose target_pose;
        target_pose.position = target_point;
        // Use the current orientation for the target pose
        target_pose.orientation = current_pose.pose.orientation;
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(current_pose.pose);
        waypoints.push_back(target_pose);
        move_group_->setMaxVelocityScalingFactor(0.05);
        move_group_->setMaxAccelerationScalingFactor(0.05);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.005;
        const double jump_threshold = 0.0;
        RCLCPP_INFO(this->get_logger(), "Planning Cartesian path...");
        double fraction = move_group_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory
        );
        if (fraction > 0.95) {
            RCLCPP_INFO(this->get_logger(), "Cartesian path computed (%.2f%% achieved)", fraction * 100.0);
            bool success = (move_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Movement completed successfully");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to execute Cartesian path");
                restoreTerminal();
                rclcpp::shutdown();
                exit(1);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute complete Cartesian path (only %.2f%% achieved)", fraction * 100.0);
            restoreTerminal();
            rclcpp::shutdown();
            exit(1);
        }
    }
    
    void configureTerminal() {
        struct termios t;
        tcgetattr(STDIN_FILENO, &old_term_);
        t = old_term_;
        t.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &t);
    }
    
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wall_subscription_;
    std::shared_ptr<rclcpp::Node> move_group_node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<std::thread> spinner_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    struct termios old_term_;
    bool initial_setup_;
    std::shared_ptr<std::thread> shutdown_thread_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto wall_painter = std::make_shared<WallPainter>();
    rclcpp::spin(wall_painter);
    wall_painter->restoreTerminal();
    rclcpp::shutdown();
    return 0;
}
