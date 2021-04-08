#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

typedef sensor_msgs::msg::PointCloud2 PC;
typedef std::chrono::steady_clock::time_point time_point; 

class Repeater : public rclcpp::Node
{
  public:
    Repeater() : Node("repeater_cpp")
    {
      pub_ = this->create_publisher<PC>("repeated_pc_cpp", 2);
      sub_ = this->create_subscription<PC>("pc_source", 2, std::bind(&Repeater::repeat_pointcloud, this, _1));
    }

  private:
    
    // Helper to get the difference in microseconds between two ROS times
    int64_t get_delta_us(time_point t1, time_point t2) const 
    {
      return std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    }

    void repeat_pointcloud(const PC::SharedPtr msg)
    {
      // Copy incoming PC and update its timestamp
      time_point t1 = std::chrono::steady_clock::now();
      PC::SharedPtr new_pc = PC::SharedPtr(new PC(*msg));
      new_pc->header.stamp = get_clock()->now();

      // Publish the copy
      time_point t2 = std::chrono::steady_clock::now();
      pub_->publish(*new_pc); 
      time_point t3 = std::chrono::steady_clock::now();

      // Log timings
      RCLCPP_INFO_STREAM(this->get_logger(), 
        "copying    took " << get_delta_us(t1, t2) << " [us]");
      RCLCPP_INFO_STREAM(this->get_logger(), 
        "publishing took " << get_delta_us(t2, t3) << " [us]");
    }
    
    rclcpp::Publisher<PC>::SharedPtr pub_;
    rclcpp::Subscription<PC>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Repeater>());
  rclcpp::shutdown();
  return 0;
}