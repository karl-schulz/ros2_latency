#include <chrono>
#include <functional>
#include <string>
// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1; 


typedef sensor_msgs::msg::PointCloud2 PC;
typedef std::chrono::steady_clock::time_point time_point; 

class Measure : public rclcpp::Node
{
  public:
    Measure() : Node("measure")
    {
      sub_cpp_ = this->create_subscription<PC>("repeated_pc_cpp", 2, std::bind(&Measure::measure_age_cpp, this, _1));
      sub_py_ = this->create_subscription<PC>("repeated_pc_py", 2, std::bind(&Measure::measure_age_py, this, _1));
    }

  private:
    
    void measure_age_cpp(const PC::SharedPtr msg)
    {
      measure_age(msg->header.stamp, "cpp");
    }
    
    void measure_age_py(const PC::SharedPtr msg)
    {
      measure_age(msg->header.stamp, "py");
    }
    
    // Helper to get the difference in microseconds between two ROS times
    int64_t get_delta_us(time_point t1, time_point t2) const 
    {
      return std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    }
    
    void measure_age(rclcpp::Time stamp, const char* method)
    {
      rclcpp::Time time_now = get_clock()->now();
      auto us = (time_now - stamp).nanoseconds() / 1e3;
      RCLCPP_INFO_STREAM(this->get_logger(), method << ": age of repeated pointcloud is " << us << " [us]");
    }
    
    rclcpp::Subscription<PC>::SharedPtr sub_cpp_;
    rclcpp::Subscription<PC>::SharedPtr sub_py_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Measure>());
  rclcpp::shutdown();
  return 0;
}