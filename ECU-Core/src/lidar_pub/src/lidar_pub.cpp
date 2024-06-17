#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include "src/CYdLidar.h"

#include <string>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;


class LidarPublisher : public rclcpp::Node
{
  public:
    LidarPublisher() : Node("lidar_pub_node")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
      timer_ = this->create_wall_timer(
        200ms, std::bind(&LidarPublisher::callback, this)
      );

      str_optval = "/dev/ttyUSB0";
      laser.setlidaropt(LidarPropSerialPort, str_optval.c_str(), str_optval.size());
      int_optval = 128000;
      laser.setlidaropt(LidarPropSerialBaudrate, &int_optval, sizeof(int));
      int_optval = TYPE_TRIANGLE;
      laser.setlidaropt(LidarPropLidarType, &int_optval, sizeof(int));
      int_optval = YDLIDAR_TYPE_SERIAL;
      laser.setlidaropt(LidarPropDeviceType, &int_optval, sizeof(int));
      float_optval = 10.0f;
      laser.setlidaropt(LidarPropScanFrequency, &float_optval, sizeof(float));
      int_optval = 5;
      laser.setlidaropt(LidarPropSampleRate, &int_optval, sizeof(int));
      bool_optval = false;
      laser.setlidaropt(LidarPropSingleChannel, &bool_optval, sizeof(bool));
      float_optval = 180.0f;
      laser.setlidaropt(LidarPropMaxAngle, &float_optval, sizeof(float));
      float_optval = -180.0f;
      laser.setlidaropt(LidarPropMinAngle, &float_optval, sizeof(float));
      float_optval = 10.0f;
      laser.setlidaropt(LidarPropMaxRange, &float_optval, sizeof(float));
      float_optval = 0.12f;
      laser.setlidaropt(LidarPropMinRange, &float_optval, sizeof(float));
      bool_optval = false;
      laser.setlidaropt(LidarPropIntenstiy, &bool_optval, sizeof(bool));

      laser.initialize();
      laser.turnOn();

      if(laser.doProcessSimple(scan))
      {
        msg.header.frame_id = "laser";
        msg.angle_min = scan.config.min_angle;
        msg.angle_max = scan.config.max_angle;
        msg.range_min = scan.config.min_range;
        msg.range_max = scan.config.max_range;
        msg.intensities.resize(0);
      }
    }

    ~LidarPublisher()
    {
      laser.turnOff();
      laser.disconnecting();
    }

  private:
    void callback()
    {
      if(laser.doProcessSimple(scan))
      {
        msg.header.stamp.sec = RCL_NS_TO_S(scan.stamp);
        msg.header.stamp.nanosec =  scan.stamp - RCL_S_TO_NS(msg.header.stamp.sec);
        msg.angle_increment = scan.config.angle_increment;
        msg.time_increment = scan.config.time_increment;
        msg.scan_time = scan.config.scan_time;

        point_size = scan.points.size();
        msg.ranges.resize(point_size);
        for(size_t i = 0; i < point_size; i++)
        {
          msg.ranges[i] = scan.points[(point_size - i - 1 + 40) % point_size].range;  // Change Orientation
        }

        publisher_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "");
      }
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string str_optval;
    int int_optval;
    float float_optval;
    bool bool_optval;

    CYdLidar laser;

    LaserScan scan;
    sensor_msgs::msg::LaserScan msg;

    size_t point_size;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPublisher>());
  rclcpp::shutdown();

  return 0;
}
