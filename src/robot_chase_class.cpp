#include "robot_chase/robot_chase_class.h"

RickandMortyChase::RickandMortyChase()
    : Node("rick_morty_chase"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(*tf_buffer_) {

    // Publisher for Rick's velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 10);

    // Timer to periodically check and update Rick's movement
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),  // 20 Hz
        std::bind(&RickandMortyChase::update_movement, this));

    // PID control gains
    kp_distance_ = 0.5; // Proportional gain for distance
    kp_yaw_ = 1.0;      // Proportional gain for angle
    max_linear_speed_ = 0.5; // Maximum linear speed
    max_angular_speed_ = 1.0; // Maximum angular speed
}

void RickandMortyChase::update_movement() {
    geometry_msgs::msg::TransformStamped transform_stamped;

    try {
        // Lookup the transform from Morty's base_link to Rick's base_link
        transform_stamped = tf_buffer_->lookupTransform(
            "rick/base_link", "morty/base_link", tf2::TimePointZero);

        double dx = transform_stamped.transform.translation.x;
        double dy = transform_stamped.transform.translation.y;

        // Calculate errors
        double error_distance = std::sqrt(dx * dx + dy * dy);
        double error_yaw = std::atan2(dy, dx);

        // PID control for velocity
        double linear_velocity = std::min(kp_distance_ * error_distance, max_linear_speed_);
        double angular_velocity = std::min(kp_yaw_ * error_yaw, max_angular_speed_);

        // Create and publish Twist message
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = linear_velocity;
        cmd_vel_msg.angular.z = angular_velocity;
        cmd_vel_pub_->publish(cmd_vel_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Distance Error: %.2f, Yaw Error: %.2f, Linear Velocity: %.2f, Angular Velocity: %.2f",
                    error_distance, error_yaw, linear_velocity, angular_velocity);

    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform Rick to Morty: %s", ex.what());
    }
}
