// STD
#include <chrono>
#include <functional>
#include <memory>
#include <string>
// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

typedef sensor_msgs::msg::PointCloud2 PC;
typedef std::chrono::steady_clock::time_point time_point; 

class Source : public rclcpp::Node
{
  public:
    Source() : Node("source")
    {
      pub_ = this->create_publisher<PC>("pc_source", 2);
      timer_ = this->create_wall_timer(1000ms, std::bind(&Source::create_pointcloud, this));

      // Each point will have _at least_ 3*4 bytes for XYZ plus 3*1 bytes for RGB = 15 bytes per point
      // Actually, the current PCL Pointcloud2 message wastes some more bytes in between
      RCLCPP_INFO_STREAM(this->get_logger(),
        "Size of created PC is about " << ((num_points_ * 32) / 1024) << " [KB]");
    }

  private:

    // Helper to get the difference in microseconds between two ROS times
    int64_t get_delta_us(time_point t1, time_point t2) const 
    {
      return std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    }

    void create_pointcloud()
    {
      time_point t1 = std::chrono::steady_clock::now();

      // Populate the a new PC with random data
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      cloud.points.reserve(num_points_);
      for (size_t i = 0; i < num_points_; ++i) {
        pcl::PointXYZRGB pt;
        pt = pcl::PointXYZRGB(255, 0, 0);  // Red
        pt.x = rand();  
        pt.y = rand();  
        pt.z = rand();  
        cloud.points.push_back(pt);
      }

      // Convert PCL Cloud to ROS message
      auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(cloud, *msg);

      // Set metadata and current timestamp
      msg->header.frame_id = "base_link";
      msg->header.stamp = get_clock()->now();
      time_point t2 = std::chrono::steady_clock::now();

      // Publish the copy
      pub_->publish(*msg); 
      time_point t3 = std::chrono::steady_clock::now();

      // Log timings
      RCLCPP_INFO_STREAM(this->get_logger(), 
        "creating   took " << get_delta_us(t1, t2) << " [us]");
      RCLCPP_INFO_STREAM(this->get_logger(), 
        "publishing took " << get_delta_us(t2, t3) << " [us]");
    }
    size_t num_points_ = declare_parameter("num_points", 640*360); 
    rclcpp::Publisher<PC>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Source>());
  rclcpp::shutdown();
  return 0;
}
