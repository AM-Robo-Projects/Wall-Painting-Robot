#include <memory>
#include <chrono>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

void configureTerminal() {
  static struct termios old, current;
  tcgetattr(0, &old);
  current = old;
  current.c_lflag &= ~ICANON;
  current.c_lflag &= ~ECHO;
  tcsetattr(0, TCSANOW, &current);
}

void restoreTerminal() {
  static struct termios old;
  tcgetattr(0, &old);
  old.c_lflag |= ICANON | ECHO;
  tcsetattr(0, TCSANOW, &old);
}

class RobotMover : public rclcpp::Node {
public:
  RobotMover() : Node("robot_mover") {
    RCLCPP_INFO(get_logger(), "Starting Robot Mover (Keyboard Control)");
    linear_step_size_ = 0.005;
    angular_step_size_ = 0.05;
    linear_increment_ = 0.001;
    angular_increment_ = 0.01;
    velocity_scaling_ = 0.1;
    acceleration_scaling_ = 0.1;
    force_feedback_enabled_ = false;
    move_group_node_ = std::make_shared<rclcpp::Node>(
      "move_group_interface_client",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    executor_.add_node(move_group_node_);
    spinner_ = std::make_shared<std::thread>([this]() { this->executor_.spin(); });
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      move_group_node_, "ur_manipulator");
    move_group_->setPlanningTime(1.0);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_->setGoalPositionTolerance(0.001);
    move_group_->setGoalOrientationTolerance(0.01);
    freedrive_enable_client_ = create_client<std_srvs::srv::Trigger>("/dashboard_client/play");
    freedrive_disable_client_ = create_client<std_srvs::srv::Trigger>("/dashboard_client/stop");
    getCurrentPose();
    printInstructions();
  }
  
  ~RobotMover() {
    if (spinner_) {
      executor_.cancel();
      if (spinner_->joinable()) {
        spinner_->join();
      }
    }
    restoreTerminal();
  }
  
  void getCurrentPose() {
    auto current_pose = move_group_->getCurrentPose();
    current_pose_ = current_pose.pose;
    tf2::Quaternion q;
    tf2::fromMsg(current_pose_.orientation, q);
    tf2::Matrix3x3(q).getRPY(current_roll_, current_pitch_, current_yaw_);
    RCLCPP_INFO(get_logger(), 
      "Robot current position: X=%.3f, Y=%.3f, Z=%.3f",
      current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
    RCLCPP_INFO(get_logger(), 
      "Robot orientation (RPY): R=%.3f, P=%.3f, Y=%.3f",
      current_roll_, current_pitch_, current_yaw_);
  }
  
  void orientFlangeDownward() {
    RCLCPP_INFO(get_logger(), "Orienting flange to point downward with minimal movement...");
    auto current_pose = move_group_->getCurrentPose().pose;
    tf2::Quaternion current_q;
    tf2::fromMsg(current_pose.orientation, current_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(current_q).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(get_logger(), "Current RPY: roll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch, yaw);
    tf2::Quaternion target_q;
    target_q.setRPY(M_PI, 0.0, yaw);
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.orientation = tf2::toMsg(target_q);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose);
    waypoints.push_back(target_pose);
    moveit_msgs::msg::RobotTrajectory trajectory;
    double eef_step = 0.01;
    double jump_threshold = 0.0;
    double saved_velocity = velocity_scaling_;
    move_group_->setMaxVelocityScalingFactor(0.1);
    std::cout << "Planning orientation adjustment..." << std::endl;
    double cartesian_success_fraction = move_group_->computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);
    if (cartesian_success_fraction > 0.95) {
      std::cout << "Executing orientation adjustment..." << std::endl;
      bool success = move_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
      if (success) {
        getCurrentPose();
        std::cout << "Flange oriented downward successfully." << std::endl;
      } else {
        std::cout << "Failed to execute orientation trajectory!" << std::endl;
      }
    } else {
      std::cout << "Cartesian planning failed. Cannot orient flange." << std::endl;
    }
    move_group_->setMaxVelocityScalingFactor(saved_velocity);
  }
  
  void toggleForceFeedback() {
    force_feedback_enabled_ = !force_feedback_enabled_;
    if (force_feedback_enabled_) {
      RCLCPP_INFO(get_logger(), "Enabling freedrive mode - you can now move the robot by hand");
      std::cout << "Freedrive mode enabled - Push the robot gently to move it" << std::endl;
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result_future = freedrive_enable_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) == 
          rclcpp::FutureReturnCode::SUCCESS) {
        auto result = result_future.get();
        if (result->success) {
          RCLCPP_INFO(get_logger(), "Freedrive mode activated successfully");
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to activate freedrive mode: %s", result->message.c_str());
          force_feedback_enabled_ = false;
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to call freedrive service");
        force_feedback_enabled_ = false;
      }
    } else {
      RCLCPP_INFO(get_logger(), "Disabling freedrive mode");
      std::cout << "Freedrive mode disabled - Robot back to normal control" << std::endl;
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto result_future = freedrive_disable_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) == 
          rclcpp::FutureReturnCode::SUCCESS) {
        auto result = result_future.get();
        if (result->success) {
          RCLCPP_INFO(get_logger(), "Freedrive mode deactivated successfully");
          getCurrentPose();
          RCLCPP_INFO(get_logger(), "Robot position synced after freedrive");
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to deactivate freedrive mode: %s", result->message.c_str());
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to call freedrive service");
      }
    }
  }
  
  void printInstructions() {
    system("clear");
    const std::string header = "=== UR5e Keyboard Control ===";
    const int header_length = header.length();
    std::stringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << "Current position: X=" << current_pose_.position.x 
       << ", Y=" << current_pose_.position.y 
       << ", Z=" << current_pose_.position.z;
    std::stringstream ss2;
    ss2 << std::fixed << std::setprecision(4);
    ss2 << "Current orientation (RPY): R=" << current_roll_
        << ", P=" << current_pitch_
        << ", Y=" << current_yaw_;
    std::stringstream ss3;
    ss3 << std::fixed << std::setprecision(4);
    ss3 << "Linear step: " << linear_step_size_ << "m (±" << linear_increment_ 
        << "m), Angular step: " << angular_step_size_ << " rad (±" << angular_increment_ << "rad)";
    int content_width = std::max({header_length, 
                                 static_cast<int>(ss.str().length()), 
                                 static_cast<int>(ss2.str().length()),
                                 static_cast<int>(ss3.str().length()),
                                 62});
    int left_padding = (content_width - header_length) / 2;
    std::string centered_header = std::string(left_padding, ' ') + header;
    std::string separator(content_width, '=');
    std::string subseparator(content_width, '-');
    std::cout << "\n" << separator << "\n";
    std::cout << centered_header << "\n";
    std::cout << ss.str() << "\n";
    std::cout << ss2.str() << "\n";
    std::cout << ss3.str() << "\n";
    std::cout << "Force Feedback: " << (force_feedback_enabled_ ? "ENABLED" : "DISABLED") << "\n";
    std::cout << subseparator << "\n";
    std::cout << "LINEAR: W/S:±X  A/D:±Y  Q/E:±Z\n";
    std::cout << "ORIENT: U/O:±Roll  I/K:±Pitch  J/L:±Yaw\n";
    std::cout << "STEP CONTROL: M/N:±LinInc  B/V:±AngInc  1/2:±LinStep  3/4:±AngStep\n";
    std::cout << "SPECIAL: F:Down  R:Sync  T:ForceToggle  X:Exit\n";
    std::cout << separator << "\n\n";
  }
  
  void run() {
    char key;
    bool exit = false;
    configureTerminal();
    RCLCPP_INFO(get_logger(), "Ready to accept keyboard commands.");
    printInstructions();
    while (!exit && rclcpp::ok()) {
      if (read(STDIN_FILENO, &key, 1) < 0) {
        continue;
      }
      geometry_msgs::msg::Pose target_pose = current_pose_;
      bool move_requested = false;
      bool orientation_changed = false;
      switch (key) {
        case 'w': case 'W':
          target_pose.position.x += linear_step_size_;
          move_requested = true;
          std::cout << "Moving +X: " << linear_step_size_ << "m\n";
          break;
        case 's': case 'S':
          target_pose.position.x -= linear_step_size_;
          move_requested = true;
          std::cout << "Moving -X: " << linear_step_size_ << "m\n";
          break;
        case 'd': case 'D':
          target_pose.position.y += linear_step_size_;
          move_requested = true;
          std::cout << "Moving +Y: " << linear_step_size_ << "m\n";
          break;
        case 'a': case 'A':
          target_pose.position.y -= linear_step_size_;
          move_requested = true;
          std::cout << "Moving -Y: " << linear_step_size_ << "m\n";
          break;
        case 'q': case 'Q':
          target_pose.position.z += linear_step_size_;
          move_requested = true;
          std::cout << "Moving +Z: " << linear_step_size_ << "m\n";
          break;
        case 'e': case 'E':
          target_pose.position.z -= linear_step_size_;
          move_requested = true;
          std::cout << "Moving -Z: " << linear_step_size_ << "m\n";
          break;
        case 'u': case 'U':
          current_roll_ += angular_step_size_;
          orientation_changed = true;
          std::cout << "Increasing roll: " << current_roll_ << " rad\n";
          break;
        case 'o': case 'O':
          current_roll_ -= angular_step_size_;
          orientation_changed = true;
          std::cout << "Decreasing roll: " << current_roll_ << " rad\n";
          break;
        case 'i': case 'I':
          current_pitch_ += angular_step_size_;
          orientation_changed = true;
          std::cout << "Increasing pitch: " << current_pitch_ << " rad\n";
          break;
        case 'k': case 'K':
          current_pitch_ -= angular_step_size_;
          orientation_changed = true;
          std::cout << "Decreasing pitch: " << current_pitch_ << " rad\n";
          break;
        case 'j': case 'J':
          current_yaw_ += angular_step_size_;
          orientation_changed = true;
          std::cout << "Increasing yaw: " << current_yaw_ << " rad\n";
          break;
        case 'l': case 'L':
          current_yaw_ -= angular_step_size_;
          orientation_changed = true;
          std::cout << "Decreasing yaw: " << current_yaw_ << " rad\n";
          break;
        case 'f': case 'F':
          if (force_feedback_enabled_) {
            std::cout << "Disabling force feedback before orientation adjustment..." << std::endl;
            toggleForceFeedback();
          }
          orientFlangeDownward();
          break;
        case 'r': case 'R':
          getCurrentPose();
          std::cout << "Position synced with robot's actual position\n";
          break;
        case 't': case 'T':
          toggleForceFeedback();
          break;
        case '1':
          linear_step_size_ += linear_increment_;
          std::cout << "Linear step size increased to: " << linear_step_size_ << "m\n";
          break;
        case '2':
          linear_step_size_ -= linear_increment_;
          if (linear_step_size_ < 0.0001) {
            linear_step_size_ = 0.0001;
            std::cout << "Minimum linear step size reached: " << linear_step_size_ << "m\n";
          } else {
            std::cout << "Linear step size decreased to: " << linear_step_size_ << "m\n";
          }
          break;
        case '3':
          angular_step_size_ += angular_increment_;
          std::cout << "Angular step size increased to: " << angular_step_size_ << " rad\n";
          break;
        case '4':
          angular_step_size_ -= angular_increment_;
          if (angular_step_size_ < 0.001) {
            angular_step_size_ = 0.001;
            std::cout << "Minimum angular step size reached: " << angular_step_size_ << " rad\n";
          } else {
            std::cout << "Angular step size decreased to: " << angular_step_size_ << " rad\n";
          }
          break;
        case 'm': case 'M':
          linear_increment_ *= 2.0;
          std::cout << "Linear increment increased to: " << linear_increment_ << "m\n";
          break;
        case 'n': case 'N':
          linear_increment_ /= 2.0;
          if (linear_increment_ < 0.0001) {
            linear_increment_ = 0.0001;
            std::cout << "Minimum linear increment reached: " << linear_increment_ << "m\n";
          } else {
            std::cout << "Linear increment decreased to: " << linear_increment_ << "m\n";
          }
          break;
        case 'b': case 'B':
          angular_increment_ *= 2.0;
          std::cout << "Angular increment increased to: " << angular_increment_ << " rad\n";
          break;
        case 'v': case 'V':
          angular_increment_ /= 2.0;
          if (angular_increment_ < 0.001) {
            angular_increment_ = 0.001;
            std::cout << "Minimum angular increment reached: " << angular_increment_ << " rad\n";
          } else {
            std::cout << "Angular increment decreased to: " << angular_increment_ << " rad\n";
          }
          break;
        case 'x': case 'X': case 27:
          exit = true;
          std::cout << "Exiting...\n";
          break;
        default:
          break;
      }
      if (orientation_changed || move_requested) {
        if (force_feedback_enabled_) {
          std::cout << "Disabling force feedback before movement..." << std::endl;
          toggleForceFeedback();
        }
      }
      if (orientation_changed) {
        tf2::Quaternion q;
        q.setRPY(current_roll_, current_pitch_, current_yaw_);
        target_pose.orientation = tf2::toMsg(q);
        move_requested = true;
      }
      if (move_requested) {
        moveToPosition(target_pose);
        printInstructions();
      } else if (key == 'f' || key == 'F' || key == 'r' || key == 'R' || 
                 key == '1' || key == '2' || key == '3' || key == '4' ||
                 key == 'm' || key == 'M' || key == 'n' || key == 'N' ||
                 key == 'b' || key == 'B' || key == 'v' || key == 'V' ||
                 key == 't' || key == 'T') {
        printInstructions();
      }
    }
  }
  
private:
  void moveToPosition(const geometry_msgs::msg::Pose& target) {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose_);
    waypoints.push_back(target);
    moveit_msgs::msg::RobotTrajectory trajectory;
    double eef_step = 0.005;
    double jump_threshold = 0.0;
    double cartesian_success_fraction = move_group_->computeCartesianPath(
      waypoints, eef_step, jump_threshold, trajectory);
    if (cartesian_success_fraction > 0.95) {
      std::cout << "Moving... ";
      std::cout.flush();
      bool success = move_group_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
      if (success) {
        std::cout << "Done.\n";
        current_pose_ = target;
      } else {
        std::cout << "Failed!\n";
        RCLCPP_ERROR(get_logger(), "Movement execution failed");
      }
    } else {
      std::cout << "Planning failed! Cannot reach target position.\n";
      RCLCPP_ERROR(get_logger(), "Cartesian planning failed (only %.2f%% complete)",
                    cartesian_success_fraction * 100.0);
    }
  }
  
  std::shared_ptr<rclcpp::Node> move_group_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<std::thread> spinner_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr freedrive_enable_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr freedrive_disable_client_;
  double linear_step_size_;
  double angular_step_size_;
  double linear_increment_;
  double angular_increment_;
  double velocity_scaling_;
  double acceleration_scaling_;
  bool force_feedback_enabled_;
  geometry_msgs::msg::Pose current_pose_;
  double current_roll_, current_pitch_, current_yaw_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotMover>();
  try {
    node->run();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
