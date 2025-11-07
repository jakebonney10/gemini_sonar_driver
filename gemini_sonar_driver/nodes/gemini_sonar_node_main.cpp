#include "gemini_sonar_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<gemini_sonar_driver::GeminiSonarNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
