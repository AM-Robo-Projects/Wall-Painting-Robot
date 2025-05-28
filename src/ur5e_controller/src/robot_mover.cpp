#include <memory>
#include <chrono>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>
#include <vector>
#include <sstream>

// Use the nlohmann/json library if available, otherwise use our own simple implementation
#ifdef HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
#else
// Simple JSON writing/parsing implementation for our specific needs
// This is a minimal implementation that only supports what we need
namespace simple_json {
  class value {
  public:
    enum type { null_type, object_type, array_type, string_type, number_type, boolean_type };
    
    value() : type_(null_type) {}
    value(const std::string& s) : type_(string_type), string_value_(s) {}
    value(double n) : type_(number_type), number_value_(n) {}
    value(bool b) : type_(boolean_type), bool_value_(b) {}
    
    value& operator[](const std::string& key) {
      type_ = object_type;
      return object_values_[key];
    }
    
    value& operator[](size_t index) {
      type_ = array_type;
      while (array_values_.size() <= index) {
        array_values_.push_back(value());
      }
      return array_values_[index];
    }
    
    void push_back(const value& v) {
      type_ = array_type;
      array_values_.push_back(v);
    }
    
    bool is_null() const { return type_ == null_type; }
    bool is_object() const { return type_ == object_type; }
    bool is_array() const { return type_ == array_type; }
    
    std::string dump(int indent = 0) const {
      std::stringstream ss;
      write(ss, indent, 0);
      return ss.str();
    }
    
    static value array() {
      value v;
      v.type_ = array_type;
      return v;
    }
    
    static value parse(std::istream& is) {
      value result;
      is >> std::ws;
      char c;
      is.get(c);
      
      if (c == '{') {  // Object
        result.type_ = object_type;
        is >> std::ws;
        if (is.peek() != '}') {
          while (true) {
            // Read key
            is >> std::ws;
            if (is.peek() != '"') throw std::runtime_error("Expected '\"' for object key");
            is.get(); // Skip opening quote
            std::string key;
            std::getline(is, key, '"');
            
            // Read colon
            is >> std::ws;
            if (is.peek() != ':') throw std::runtime_error("Expected ':' after object key");
            is.get(); // Skip colon
            
            // Read value
            result.object_values_[key] = parse(is);
            
            // Check if there are more entries
            is >> std::ws;
            c = is.get();
            if (c == '}') break;
            if (c != ',') throw std::runtime_error("Expected ',' or '}' after object entry");
          }
        } else {
          is.get(); // Skip closing brace
        }
      } else if (c == '[') {  // Array
        result.type_ = array_type;
        is >> std::ws;
        if (is.peek() != ']') {
          while (true) {
            result.array_values_.push_back(parse(is));
            
            // Check if there are more entries
            is >> std::ws;
            c = is.get();
            if (c == ']') break;
            if (c != ',') throw std::runtime_error("Expected ',' or ']' after array element");
          }
        } else {
          is.get(); // Skip closing bracket
        }
      } else if (c == '"') {  // String
        result.type_ = string_type;
        std::getline(is, result.string_value_, '"');
      } else if (isdigit(c) || c == '-') {  // Number
        is.unget();
        result.type_ = number_type;
        is >> result.number_value_;
      } else if (c == 't' || c == 'f') {  // Boolean
        is.unget();
        std::string val;
        if (c == 't') {
          val.resize(4);
          is.read(&val[0], 4);
          if (val != "true") throw std::runtime_error("Expected 'true' for boolean value");
          result.type_ = boolean_type;
          result.bool_value_ = true;
        } else {
          val.resize(5);
          is.read(&val[0], 5);
          if (val != "false") throw std::runtime_error("Expected 'false' for boolean value");
          result.type_ = boolean_type;
          result.bool_value_ = false;
        }
      } else {
        throw std::runtime_error(std::string("Unexpected character in JSON: ") + c);
      }
      
      return result;
    }
    
    operator std::string() const {
      if (type_ != string_type) throw std::runtime_error("Not a string");
      return string_value_;
    }
    
    operator double() const {
      if (type_ != number_type) throw std::runtime_error("Not a number");
      return number_value_;
    }
    
    size_t size() const {
      if (type_ != array_type) throw std::runtime_error("Not an array");
      return array_values_.size();
    }
    
  private:
    void write(std::ostream& os, int indent = 0, int level = 0) const {
      std::string spaces(level * indent, ' ');
      switch (type_) {
        case null_type:
          os << "null";
          break;
        case object_type:
          if (object_values_.empty()) {
            os << "{}";
          } else {
            os << "{";
            if (indent > 0) os << "\n";
            bool first = true;
            for (const auto& kv : object_values_) {
              if (!first) {
                os << ",";
                if (indent > 0) os << "\n";
              }
              first = false;
              if (indent > 0) os << spaces << std::string(indent, ' ');
              os << "\"" << kv.first << "\": ";
              kv.second.write(os, indent, level + 1);
            }
            if (indent > 0) os << "\n" << spaces;
            os << "}";
          }
          break;
        case array_type:
          if (array_values_.empty()) {
            os << "[]";
          } else {
            os << "[";
            if (indent > 0) os << "\n";
            bool first = true;
            for (const auto& v : array_values_) {
              if (!first) {
                os << ",";
                if (indent > 0) os << "\n";
              }
              first = false;
              if (indent > 0) os << spaces << std::string(indent, ' ');
              v.write(os, indent, level + 1);
            }
            if (indent > 0) os << "\n" << spaces;
            os << "]";
          }
          break;
        case string_type:
          os << "\"" << string_value_ << "\"";
          break;
        case number_type:
          os << std::fixed << std::setprecision(10) << number_value_;
          break;
        case boolean_type:
          os << (bool_value_ ? "true" : "false");
          break;
      }
    }
    
    type type_;
    std::string string_value_;
    double number_value_ = 0.0;
    bool bool_value_ = false;
    std::map<std::string, value> object_values_;
    std::vector<value> array_values_;
  };
}  // namespace simple_json

// Define an alias for the JSON implementation we're using
#define JSON_NS simple_json
#endif

#ifndef JSON_NS
// If we have the nlohmann library
#define JSON_NS nlohmann
#endif

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

struct SavedPoint {
  std::string name;
  // Cartesian pose
  geometry_msgs::msg::Pose pose;
  // Joint values
  std::vector<double> joint_values;
};

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
    createSavePointDirectory();
    loadSavedPoints();
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
    current_joint_values_ = move_group_->getCurrentJointValues();
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
    std::cout << "Saved Points: " << saved_points_.size() << "\n";
    std::cout << subseparator << "\n";
    std::cout << "LINEAR: W/S:±X  A/D:±Y  Q/E:±Z\n";
    std::cout << "ORIENT: U/O:±Roll  I/K:±Pitch  J/L:±Yaw\n";
    std::cout << "STEP CONTROL: M/N:±LinInc  B/V:±AngInc  1/2:±LinStep  3/4:±AngStep\n";
    std::cout << "SAVE POINTS: P:Save  G:GoTo  Y:List\n";
    std::cout << "SPECIAL: F:Down  R:Sync  T:ForceToggle  H:Home  X:Exit\n";
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
      
      bool move_requested = false;
      
      // Convert current pose to Eigen for TCP-relative manipulations
      Eigen::Isometry3d current_pose_eigen;
      tf2::fromMsg(current_pose_, current_pose_eigen);
      
      switch (key) {
        case 'w': case 'W':
          // Move in +X direction in tool frame
          current_pose_eigen = current_pose_eigen * Eigen::Translation3d(linear_step_size_, 0, 0);
          move_requested = true;
          std::cout << "Moving +X (tool frame): " << linear_step_size_ << "m\n";
          break;
        case 's': case 'S':
          // Move in -X direction in tool frame
          current_pose_eigen = current_pose_eigen * Eigen::Translation3d(-linear_step_size_, 0, 0);
          move_requested = true;
          std::cout << "Moving -X (tool frame): " << linear_step_size_ << "m\n";
          break;
        case 'd': case 'D':
          // Move in +Y direction in tool frame
          current_pose_eigen = current_pose_eigen * Eigen::Translation3d(0, linear_step_size_, 0);
          move_requested = true;
          std::cout << "Moving +Y (tool frame): " << linear_step_size_ << "m\n";
          break;
        case 'a': case 'A':
          // Move in -Y direction in tool frame
          current_pose_eigen = current_pose_eigen * Eigen::Translation3d(0, -linear_step_size_, 0);
          move_requested = true;
          std::cout << "Moving -Y (tool frame): " << linear_step_size_ << "m\n";
          break;
        case 'q': case 'Q':
          // Move in +Z direction in tool frame
          current_pose_eigen = current_pose_eigen * Eigen::Translation3d(0, 0, linear_step_size_);
          move_requested = true;
          std::cout << "Moving +Z (tool frame): " << linear_step_size_ << "m\n";
          break;
        case 'e': case 'E':
          // Move in -Z direction in tool frame
          current_pose_eigen = current_pose_eigen * Eigen::Translation3d(0, 0, -linear_step_size_);
          move_requested = true;
          std::cout << "Moving -Z (tool frame): " << linear_step_size_ << "m\n";
          break;
        case 'u': case 'U':
          // Rotate around tool X-axis (roll)
          current_pose_eigen = current_pose_eigen * 
                              Eigen::AngleAxisd(angular_step_size_, Eigen::Vector3d::UnitX());
          move_requested = true;
          std::cout << "Rotating around tool X-axis: " << angular_step_size_ << " rad\n";
          break;
        case 'o': case 'O':
          // Rotate around tool X-axis (roll) negative
          current_pose_eigen = current_pose_eigen * 
                              Eigen::AngleAxisd(-angular_step_size_, Eigen::Vector3d::UnitX());
          move_requested = true;
          std::cout << "Rotating around tool X-axis: " << -angular_step_size_ << " rad\n";
          break;
        case 'i': case 'I':
          // Rotate around tool Y-axis (pitch)
          current_pose_eigen = current_pose_eigen * 
                              Eigen::AngleAxisd(angular_step_size_, Eigen::Vector3d::UnitY());
          move_requested = true;
          std::cout << "Rotating around tool Y-axis: " << angular_step_size_ << " rad\n";
          break;
        case 'k': case 'K':
          // Rotate around tool Y-axis (pitch) negative
          current_pose_eigen = current_pose_eigen * 
                              Eigen::AngleAxisd(-angular_step_size_, Eigen::Vector3d::UnitY());
          move_requested = true;
          std::cout << "Rotating around tool Y-axis: " << -angular_step_size_ << " rad\n";
          break;
        case 'j': case 'J':
          // Rotate around tool Z-axis (yaw)
          current_pose_eigen = current_pose_eigen * 
                              Eigen::AngleAxisd(angular_step_size_, Eigen::Vector3d::UnitZ());
          move_requested = true;
          std::cout << "Rotating around tool Z-axis: " << angular_step_size_ << " rad\n";
          break;
        case 'l': case 'L':
          // Rotate around tool Z-axis (yaw) negative
          current_pose_eigen = current_pose_eigen * 
                              Eigen::AngleAxisd(-angular_step_size_, Eigen::Vector3d::UnitZ());
          move_requested = true;
          std::cout << "Rotating around tool Z-axis: " << -angular_step_size_ << " rad\n";
          break;
        
        // Keep all other commands the same
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
        case 'h': case 'H': {
          if (force_feedback_enabled_) {
            std::cout << "Disabling force feedback before moving home..." << std::endl;
            toggleForceFeedback();
          }
          std::cout << "Moving to HOME position...\n";
          geometry_msgs::msg::Pose home_pose;
          home_pose.position.x = 0.00;
          home_pose.position.y = 0.33;
          home_pose.position.z = 0.40;
          tf2::Quaternion q;
          q.setRPY(3.14, 0.0, 0.0);
          home_pose.orientation = tf2::toMsg(q);
          moveToPosition(home_pose);
          getCurrentPose();
          printInstructions();
          break;
        }
        case 'p': case 'P':
          // Save current point
          saveCurrentPoint();
          break;
        case 'g': case 'G':
          // Go to saved point
          gotoSavedPoint();
          break;
        case 'y': case 'Y':
          // List saved points
          listSavedPoints();
          break;
        case 'x': case 'X': case 27:
          exit = true;
          std::cout << "Exiting...\n";
          break;
        default:
          break;
      }
      
      if (move_requested) {
        if (force_feedback_enabled_) {
          std::cout << "Disabling force feedback before movement..." << std::endl;
          toggleForceFeedback();
        }
        
        // Convert Eigen transformation back to geometry_msgs::Pose
        geometry_msgs::msg::Pose target_pose = tf2::toMsg(current_pose_eigen);
        
        // Update current orientation values for display
        tf2::Quaternion q;
        tf2::fromMsg(target_pose.orientation, q);
        tf2::Matrix3x3(q).getRPY(current_roll_, current_pitch_, current_yaw_);
        
        moveToPosition(target_pose);
        printInstructions();
      } else if (key == 'f' || key == 'F' || key == 'r' || key == 'R' || 
                key == '1' || key == '2' || key == '3' || key == '4' ||
                key == 'm' || key == 'M' || key == 'n' || key == 'N' ||
                key == 'b' || key == 'B' || key == 'v' || key == 'V' ||
                key == 't' || key == 'T' || key == 'p' || key == 'P' ||
                key == 'g' || key == 'G' || key == 'y' || key == 'Y') {
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
        // Update joint values after cartesian movement
        current_joint_values_ = move_group_->getCurrentJointValues();
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
  
  void moveToJointPosition(const std::vector<double>& joint_values) {
    std::cout << "Moving to joint position... ";
    std::cout.flush();
    move_group_->setJointValueTarget(joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (success) {
      success = move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      if (success) {
        std::cout << "Done.\n";
        getCurrentPose(); // Update current pose and joint values
      } else {
        std::cout << "Failed!\n";
        RCLCPP_ERROR(get_logger(), "Joint movement execution failed");
      }
    } else {
      std::cout << "Joint planning failed!\n";
      RCLCPP_ERROR(get_logger(), "Joint planning failed");
    }
  }

  void createSavePointDirectory() {
    try {
      // Use ROS 2 package discovery system to find the package directory
      std::string package_name = "ur5e_controller";
      std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
      
      // Create the saved_points directory within the package directory
      save_points_dir_ = fs::path(package_share_directory) / "saved_points";
      
      RCLCPP_INFO(get_logger(), "Using directory for saved points: %s", 
                  save_points_dir_.string().c_str());
      
      // Create the directory if it doesn't exist
      if (!fs::exists(save_points_dir_)) {
        fs::create_directories(save_points_dir_);
        RCLCPP_INFO(get_logger(), "Created directory for saved points: %s", 
                    save_points_dir_.string().c_str());
      }

      // Define the JSON file path
      points_file_path_ = save_points_dir_ / "saved_points.json";
      
      // Initialize empty JSON file if it doesn't exist
      if (!fs::exists(points_file_path_)) {
        #ifdef HAVE_NLOHMANN_JSON
        nlohmann::json empty_points = nlohmann::json::array();
        #else
        JSON_NS::value empty_points = JSON_NS::value::array();
        #endif
        
        std::ofstream file(points_file_path_);
        if (file.is_open()) {
          file << empty_points.dump(2); // Pretty print with 2-space indent
          file.close();
          RCLCPP_INFO(get_logger(), "Created empty saved points file: %s", 
                      points_file_path_.string().c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to create saved points file: %s", 
                       points_file_path_.string().c_str());
        }
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to locate or create saved points directory: %s", e.what());
      throw std::runtime_error("Cannot initialize saved points directory. Check if package name is correct.");
    }
  }

  void saveCurrentPoint() {
    restoreTerminal();
    std::cout << "\nEnter name for this position: ";
    std::string point_name;
    std::getline(std::cin, point_name);
    
    if (point_name.empty()) {
      std::cout << "Save cancelled - empty name provided.\n";
      configureTerminal();
      return;
    }
    
    // Check if name already exists
    for (auto& pt : saved_points_) {
      if (pt.name == point_name) {
        std::cout << "A point with this name already exists. Overwrite? (y/n): ";
        char choice;
        std::cin >> choice;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        if (choice != 'y' && choice != 'Y') {
          std::cout << "Save cancelled.\n";
          configureTerminal();
          return;
        }
        // Remove the existing point
        auto it = std::find_if(saved_points_.begin(), saved_points_.end(),
                         [&point_name](const SavedPoint& p) { return p.name == point_name; });
        if (it != saved_points_.end()) {
          saved_points_.erase(it);
        }
        break;
      }
    }
    
    SavedPoint new_point;
    new_point.name = point_name;
    new_point.pose = current_pose_;
    new_point.joint_values = current_joint_values_;
    saved_points_.push_back(new_point);
    
    // Save to JSON file
    savePointsToJsonFile();
    
    std::cout << "Position saved as '" << point_name << "'.\n";
    configureTerminal();
  }

  void savePointsToJsonFile() {
    #ifdef HAVE_NLOHMANN_JSON
    nlohmann::json points_array = nlohmann::json::array();
    #else
    JSON_NS::value points_array = JSON_NS::value::array();
    #endif
  
    for (size_t i = 0; i < saved_points_.size(); i++) {
      const auto& point = saved_points_[i];
      
      #ifdef HAVE_NLOHMANN_JSON
      nlohmann::json point_json;
      #else
      JSON_NS::value point_json;
      #endif
    
      point_json["name"] = point.name;
      point_json["pose"]["position"]["x"] = point.pose.position.x;
      point_json["pose"]["position"]["y"] = point.pose.position.y;
      point_json["pose"]["position"]["z"] = point.pose.position.z;
      point_json["pose"]["orientation"]["x"] = point.pose.orientation.x;
      point_json["pose"]["orientation"]["y"] = point.pose.orientation.y;
      point_json["pose"]["orientation"]["z"] = point.pose.orientation.z;
      point_json["pose"]["orientation"]["w"] = point.pose.orientation.w;
    
      #ifdef HAVE_NLOHMANN_JSON
      nlohmann::json joint_values_array = nlohmann::json::array();
      #else
      JSON_NS::value joint_values_array = JSON_NS::value::array();
      #endif
    
      for (size_t j = 0; j < point.joint_values.size(); j++) {
        joint_values_array.push_back(point.joint_values[j]);
      }
      point_json["joint_values"] = joint_values_array;
    
      #ifdef HAVE_NLOHMANN_JSON
      points_array.push_back(point_json);
      #else
      points_array.push_back(point_json);
      #endif
    }
    
    std::ofstream file(points_file_path_);
    if (file.is_open()) {
      file << points_array.dump(2); // Pretty print with 2-space indent
      file.close();
      RCLCPP_INFO(get_logger(), "Saved %zu points to JSON file: %s", 
                  saved_points_.size(), points_file_path_.string().c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to save points to JSON file: %s", 
                   points_file_path_.string().c_str());
    }
  }

  void loadSavedPoints() {
    saved_points_.clear();
    
    try {
      if (!fs::exists(points_file_path_)) {
        RCLCPP_INFO(get_logger(), "No saved points file found at: %s", 
                    points_file_path_.string().c_str());
        return;
      }
      
      std::ifstream file(points_file_path_);
      if (!file.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open saved points file: %s", 
                     points_file_path_.string().c_str());
        return;
      }
      
      #ifdef HAVE_NLOHMANN_JSON
      nlohmann::json points_array;
      try {
        file >> points_array;
      } catch (const nlohmann::json::parse_error& e) {
        RCLCPP_ERROR(get_logger(), "Error parsing JSON: %s", e.what());
        return;
      }
      #else
      JSON_NS::value points_array;
      try {
        points_array = JSON_NS::value::parse(file);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error parsing JSON: %s", e.what());
        return;
      }
      #endif
    
      #ifdef HAVE_NLOHMANN_JSON
      for (const auto& point_json : points_array) {
      #else
      for (size_t i = 0; i < points_array.size(); i++) {
        auto& point_json = points_array[i];
      #endif
        SavedPoint point;
        point.name = point_json["name"];
        
        point.pose.position.x = point_json["pose"]["position"]["x"];
        point.pose.position.y = point_json["pose"]["position"]["y"];
        point.pose.position.z = point_json["pose"]["position"]["z"];
        
        point.pose.orientation.x = point_json["pose"]["orientation"]["x"];
        point.pose.orientation.y = point_json["pose"]["orientation"]["y"];
        point.pose.orientation.z = point_json["pose"]["orientation"]["z"];
        point.pose.orientation.w = point_json["pose"]["orientation"]["w"];
        
        #ifdef HAVE_NLOHMANN_JSON
        for (const auto& joint_value : point_json["joint_values"]) {
          point.joint_values.push_back(joint_value);
        }
        #else
        auto& joint_values = point_json["joint_values"];
        for (size_t j = 0; j < joint_values.size(); j++) {
          point.joint_values.push_back(double(joint_values[j]));
        }
        #endif
        
        saved_points_.push_back(point);
        RCLCPP_INFO(get_logger(), "Loaded saved point: %s", point.name.c_str());
      }
      
      RCLCPP_INFO(get_logger(), "Loaded %zu saved points", saved_points_.size());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Error loading saved points: %s", e.what());
    }
  }

  void listSavedPoints() {
    system("clear");
    std::cout << "\n=== Saved Points (" << saved_points_.size() << ") ===\n\n";
    
    if (saved_points_.empty()) {
      std::cout << "No saved points found.\n";
    } else {
      for (size_t i = 0; i < saved_points_.size(); ++i) {
        const auto& point = saved_points_[i];
        double roll, pitch, yaw;
        tf2::Quaternion q;
        tf2::fromMsg(point.pose.orientation, q);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        std::cout << i+1 << ". " << point.name << "\n";
        std::cout << "   Pose: X=" << std::fixed << std::setprecision(4) << point.pose.position.x 
                  << " Y=" << point.pose.position.y 
                  << " Z=" << point.pose.position.z << "\n";
        std::cout << "   RPY: R=" << roll << " P=" << pitch << " Y=" << yaw << "\n";
        std::cout << "   Joints: ";
        for (size_t j = 0; j < point.joint_values.size(); ++j) {
          std::cout << std::fixed << std::setprecision(4) << point.joint_values[j];
          if (j < point.joint_values.size() - 1) {
            std::cout << ", ";
          }
        }
        std::cout << "\n\n";
      }
    }
    
    std::cout << "\nPress any key to continue...";
    char c;
    read(STDIN_FILENO, &c, 1);
  }

  void gotoSavedPoint() {
    if (saved_points_.empty()) {
      std::cout << "No saved points available.\n";
      return;
    }
    
    system("clear");
    std::cout << "\n=== Go to Saved Point ===\n\n";
    for (size_t i = 0; i < saved_points_.size(); ++i) {
      std::cout << i+1 << ". " << saved_points_[i].name << "\n";
    }
    
    std::cout << "\nEnter point number (1-" << saved_points_.size() << ") or 0 to cancel: ";
    restoreTerminal();
    
    std::string input;
    std::getline(std::cin, input);
    
    int choice = 0;
    try {
      choice = std::stoi(input);
    } catch (...) {
      std::cout << "Invalid input. Operation cancelled.\n";
      configureTerminal();
      return;
    }
    
    if (choice < 1 || choice > static_cast<int>(saved_points_.size())) {
      std::cout << "Invalid selection. Operation cancelled.\n";
      configureTerminal();
      return;
    }
    
    const SavedPoint& selected_point = saved_points_[choice-1];
    
    std::cout << "Selected point: " << selected_point.name << "\n";
    std::cout << "Move using [C]artesian coordinates or [J]oint values? (C/J): ";
    std::string move_mode;
    std::getline(std::cin, move_mode);
    
    configureTerminal();
    
    // Disable force feedback if enabled
    if (force_feedback_enabled_) {
      std::cout << "Disabling force feedback before movement..." << std::endl;
      toggleForceFeedback();
    }
    
    if (!move_mode.empty() && (move_mode[0] == 'j' || move_mode[0] == 'J')) {
      std::cout << "Moving to point '" << selected_point.name << "' using joint values...\n";
      moveToJointPosition(selected_point.joint_values);
    } else {
      std::cout << "Moving to point '" << selected_point.name << "' using cartesian coordinates...\n";
      moveToPosition(selected_point.pose);
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
  std::vector<double> current_joint_values_;
  double current_roll_, current_pitch_, current_yaw_;
  
  fs::path save_points_dir_;
  fs::path points_file_path_;  // New: Path to the JSON file
  std::vector<SavedPoint> saved_points_;
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
