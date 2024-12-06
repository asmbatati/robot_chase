#include "robot_chase/robot_chase_class.h"

RickandMortyChase::RickandMortyChase()
    : Node("rick_morty_chase"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(*tf_buffer_) {
    // Initialize publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 10);

    // Initialize subscription to Morty's velocity topic
    morty_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "morty/cmd_vel", 10, std::bind(&RickandMortyChase::morty_velocity_callback, this, std::placeholders::_1));

    // Initialize subscription to Rick's velocity topic
    rick_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "rick/odom", 10, std::bind(&RickandMortyChase::rick_velocity_callback, this, std::placeholders::_1));

    // Timer for periodic updates
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&RickandMortyChase::update_movement, this));

    // PID parameters
    kp_distance_ = 0.5;
    kp_yaw_ = 1.0;
    max_linear_speed_ = 0.5;
    max_angular_speed_ = 1.0;
}

void RickandMortyChase::update_movement() {
    geometry_msgs::msg::TransformStamped transform_stamped;

    try {
        // Lookup transform from Morty to Rick
        transform_stamped = tf_buffer_->lookupTransform(
            "rick/base_link", "morty/base_link", tf2::TimePointZero);

        // Calculate position errors
        double dx = transform_stamped.transform.translation.x;
        double dy = transform_stamped.transform.translation.y;
        double error_distance = std::sqrt(dx * dx + dy * dy);
        double error_yaw = std::atan2(dy, dx);
        double error_yaw_degrees = error_yaw * (180.0 / M_PI);

        // Calculate velocities using proportional control
        double linear_velocity = std::min(kp_distance_ * error_distance, max_linear_speed_);
        double angular_velocity = std::min(kp_yaw_ * error_yaw, max_angular_speed_);

        // Create and publish Twist message
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = linear_velocity;
        cmd_vel_msg.angular.z = angular_velocity;
        cmd_vel_pub_->publish(cmd_vel_msg);

        // Check Morty's state and distance
        if (is_morty_stopped() && error_distance < 0.6) {

            handle_bump();

            if (has_rick_bumped()) {
                stop();
                RCLCPP_INFO(this->get_logger(), "Rick has bumped into Morty.");
                return;
            }

            return;
        }

        // RCLCPP_INFO(this->get_logger(),
        //             "Distance Error: %.2f, Yaw Error: %.2f (Degrees: %.2f), "
        //             "Linear Velocity: %.2f, Angular Velocity: %.2f",
        //             error_distance, error_yaw, error_yaw_degrees,
        //             linear_velocity, angular_velocity);

    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform Rick to Morty: %s", ex.what());
    }
}

void RickandMortyChase::handle_bump() {
    geometry_msgs::msg::Twist cmd_vel_msg;

    // Bump behavior: slight linear velocity
    cmd_vel_msg.linear.x = 0.2;
    cmd_vel_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel_msg);

}

void RickandMortyChase::stop() {
    geometry_msgs::msg::Twist cmd_vel_msg;

    // Stop after bump
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel_msg);

    RCLCPP_INFO(this->get_logger(), "Rick has Stopped.");
}

bool RickandMortyChase::is_morty_stopped() {
    // Check if Morty's velocity is below a threshold
    double linear_speed = std::sqrt(morty_velocity_.linear.x * morty_velocity_.linear.x +
                                    morty_velocity_.linear.y * morty_velocity_.linear.y);

    const double linear_threshold = 0.05;

    return linear_speed < linear_threshold;
}

bool RickandMortyChase::has_rick_bumped() {
    // Check if Rick's velocity is below a threshold
    double linear_speed = std::sqrt(rick_velocity_.x * rick_velocity_.x +
                                    rick_velocity_.y * rick_velocity_.y);

    const double linear_threshold = 0.05;

    return linear_speed < linear_threshold;
}

void RickandMortyChase::morty_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    morty_velocity_ = *msg;
}

void RickandMortyChase::rick_velocity_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    rick_velocity_ = msg->twist.twist.linear;
}