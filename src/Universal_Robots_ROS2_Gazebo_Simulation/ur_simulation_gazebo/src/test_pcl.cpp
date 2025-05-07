#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

class PCLTest : public rclcpp::Node
{
public:
    PCLTest() : Node("pcl_test")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "livox/points", 10, 
            std::bind(&PCLTest::cloud_callback, this, std::placeholders::_1));
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        RCLCPP_INFO(this->get_logger(), "Received cloud with %zu points", cloud->size());
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}