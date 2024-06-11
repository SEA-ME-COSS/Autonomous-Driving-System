#include "control.hpp"

Control::Control() : rclcpp::Node("vehicle_control") {
    double k = 1.5;
    double ks = 70.2;

    this->controller = Stanley(k, ks);

    // Initialize CAN receiver
    this->can_sender = std::make_unique<CANSender>("src/stanley/include/stanley/utils/example.dbc", "vcan0");

    // Subscribe
    path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        "/pathplanner/path", 10, std::bind(&Control::path_callback, this,  std::placeholders::_1));
    velocity_subscription_ = this->create_subscription<example_interfaces::msg::Float64>(
        "/pathplanner/throttle", 10, std::bind(&Control::velocity_callback, this, std::placeholders::_1));
    pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/piracer/odom", 10, std::bind(&Control::pose_callback, this,  std::placeholders::_1));
    
    // Publish
    drive_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/piracer/cmd_vel", 10);

    publisher_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Control::publisher_timer_callback, this)
    );
}

void Control::path_callback(const nav_msgs::msg::Path::SharedPtr path_msg) {
    this->pathValid = false;
    this->refPoses.clear();
    Path refPose;
    for (size_t i = 0; i < path_msg->poses.size(); ++i) {
        refPose.x = path_msg->poses[i].pose.position.x;
        refPose.y = path_msg->poses[i].pose.position.y;
        refPose.yaw = this->quat_to_yaw(path_msg->poses[i].pose.orientation);
        this->refPoses.push_back(refPose);
    }
    this->pathValid = true;
}

void Control::velocity_callback(const example_interfaces::msg::Float64::SharedPtr velocity_msg) {
    this->velocityValid = false;
    std::cout << "target speed : " << velocity_msg->data << std::endl;
    this->target_velocity = velocity_msg->data;
    this->velocityValid = true;
}

void Control::pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg) {
    this->poseValid = false;
    this->currPose.x = pose_msg->pose.pose.position.x * 100 + 33;  // [cm]
    this->currPose.y = pose_msg->pose.pose.position.y * 100 + 50;  // [cm]
    
    tf2::Quaternion q(
      pose_msg->pose.pose.orientation.x,
      pose_msg->pose.pose.orientation.y,
      pose_msg->pose.pose.orientation.z,
      pose_msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);  // [rad]
    this->currPose.yaw = yaw;

    this->currPose.v = sqrt(pow(pose_msg->twist.twist.linear.x, 2)
                        + pow(pose_msg->twist.twist.linear.y, 2)) * 5;  // [cm/s]
    this->poseValid = true;
}

void Control::publisher_timer_callback() {
    if (this->refPoses.empty() || !this->poseValid || !this->pathValid) { std::cout << "Message Receive Error" << std::endl; return;}

    this->controller.stanley_control(this->refPoses, this->target_velocity, this->currPose.x, this->currPose.y, this->currPose.yaw, this->currPose.v);

    this->speedCommand = this->controller.getThrottle();
    // this->speedCommand = 0.5;
    this->steerCommand = this->controller.getDelta();

    Path lastRefPose = this->refPoses.back();
    if ((std::pow(this->currPose.x - lastRefPose.x, 2) + std::pow(this->currPose.y - lastRefPose.y, 2)) < 40.0) {
        std::cout << "Finished!!!" << std::endl;
        this->speedCommand = 0.0;
    }

    this->publish_drive(this->speedCommand, this->steerCommand);

    this->can_sender->sendSpeedMessage(this->speedCommand, 0);
    this->can_sender->sendSteerMessage(this->steerCommand, 1);
}

void Control::publish_drive(float speed, float steer) {
    auto drive_msg = std::make_unique<geometry_msgs::msg::Twist>();
    drive_msg->linear.x = speed;
    drive_msg->angular.z = steer;
    std::cout << "Publish Throttle : " << speed << " Publish Steering : " << steer << std::endl;
    this->drive_publisher_->publish(std::move(drive_msg));
}

float Control::quat_to_yaw(const geometry_msgs::msg::Quaternion quat) {
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(quat, tf2_quat);
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3 matrix(tf2_quat);
    matrix.getRPY(roll, pitch, yaw);
    return yaw;
}
