#ifndef RICK_AND_MORTY_CHASE_HPP
#define RICK_AND_MORTY_CHASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

class RickandMortyChase : public rclcpp::Node {
public:
    RickandMortyChase();

private:
    void update_movement();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    // PID parameters
    double kp_distance_;
    double kp_yaw_;
    double max_linear_speed_;
    double max_angular_speed_;
};

#endif // RICK_AND_MORTY_CHASE_HPP
