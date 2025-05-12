#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>

class PCLTest : public rclcpp::Node
{
public:
    PCLTest() : Node("pcl_test"), cloud_count_(0)
    {
        // Create pcd_files directory if it doesn't exist
        std::filesystem::create_directory("pcd_files");
        
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/mid360_PointCloud2", 10, 
            std::bind(&PCLTest::cloud_callback, this, std::placeholders::_1));
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        // Generate filename with timestamp
        std::string filename = "pcd_files/cloud_" + 
                             std::to_string(cloud_count_++) + ".pcd";
        
        // Save the point cloud
        if (pcl::io::savePCDFileBinary(filename, *cloud) == 0)
        {
            RCLCPP_INFO(this->get_logger(), 
                       "Saved %zu points to %s", 
                       cloud->size(), 
                       filename.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), 
                        "Failed to save point cloud to %s", 
                        filename.c_str());
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    int cloud_count_;  // Counter for unique filenames
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}