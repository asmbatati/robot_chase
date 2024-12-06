#include "robot_chase/robot_chase_class.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RickandMortyChase>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
