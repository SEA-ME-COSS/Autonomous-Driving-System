#pragma once

#include "utils/loader.hpp"
#include "map/map.hpp"
#include "planner/planner.hpp"
#include "decision_making/decision_making.hpp"

#include "data_structure/ros2_msg_struct.h"

#include "rclcpp/rclcpp.hpp"

#include "vision_msgs/msg/classification2_d.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <string>
#include <cmath>
#include <vector>
#include <deque>
#include <utility>

class PathPlanning : public rclcpp::Node {
public:
    PathPlanning();

private:
    Map *map;
    Planner planner;
    DecisionMaking decision_making;

    std::vector<Sign> signs;
    Pose pose;

    vision_msgs::msg::Classification2D::SharedPtr sign_msg;
    nav_msgs::msg::Odometry::SharedPtr pose_msg;
    std_msgs::msg::Int8::SharedPtr start_msg;

    bool use_sign;
    bool use_pose;

    bool use_start;

    std::vector<std::vector<double>> path;
    double throttle;
    int state;
    int start_available;

    double normal_throttle;

    std::deque<std::pair<float, std::vector<std::vector<double>>>> routes;

    rclcpp::Subscription<vision_msgs::msg::Classification2D>::SharedPtr sign_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr start_subscription_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr throttle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr state_publisher_;

    rclcpp::TimerBase::SharedPtr publisher_timer_;

    void sign_callback(const vision_msgs::msg::Classification2D::SharedPtr sign_msg);
    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg);
    void start_callback(const std_msgs::msg::Int8::SharedPtr start_msg);

    void publisher_timer_callback();

    bool isUseMessageValid();
    void updateUseMessages();
    void publish_path();
    void publish_throttle();
    void publish_state();

    void update_sign();
    void update_pose();
    void update_start();

    void addPose(nav_msgs::msg::Path& path_msg, std::vector<double> pose);
};