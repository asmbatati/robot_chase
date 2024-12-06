#ifndef ROBOT_CHASE_CLASS_HPP_
#define ROBOT_CHASE_CLASS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

class RickandMortyChase : public rclcpp::Node {
public:
    RickandMortyChase();

private:
    void update_movement();
    void handle_bump();
    void stop();
    bool is_morty_stopped();
    bool has_rick_bumped();
    void morty_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void rick_velocity_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // ROS entities
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr morty_velocity_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rick_odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // PID parameters
    double kp_distance_;
    double kp_yaw_;
    double max_linear_speed_;
    double max_angular_speed_;

    // Morty's velocity
    geometry_msgs::msg::Twist morty_velocity_;
    // Rick's velocity
    geometry_msgs::msg::Vector3 rick_velocity_;
};


#endif  // ROBOT_CHASE_CLASS_HPP_