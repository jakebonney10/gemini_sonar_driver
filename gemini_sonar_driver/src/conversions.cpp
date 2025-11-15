#include "gemini_sonar_driver/conversions.hpp"
#include <algorithm>
#include <cmath>

NS_HEAD

namespace conversions
{

marine_acoustic_msgs::msg::PingInfo createPingInfo(const ConversionParameters& params)
{
    marine_acoustic_msgs::msg::PingInfo ping_info;
    
    // Convert frequency from kHz to Hz
    ping_info.frequency = params.frequency_khz * 1000.0;
    ping_info.sound_speed = params.sound_speed_ms;
    
    return ping_info;
}

marine_acoustic_msgs::msg::RawSonarImage::SharedPtr createRawSonarImage(
    const std::vector<std::vector<uint8_t>>& beam_data,
    const ConversionParameters& params,
    const rclcpp::Time& timestamp)
{
    if (beam_data.empty())
    {
        return nullptr;
    }
    
    auto msg = std::make_shared<marine_acoustic_msgs::msg::RawSonarImage>();
    
    // Header
    msg->header.stamp = timestamp;
    msg->header.frame_id = params.frame_id;
    
    // Ping info
    msg->ping_info = createPingInfo(params);

    // Image data
    msg->image = createSonarImageData(beam_data, marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT8);
    
    return msg;
}

marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr createProjectedSonarImage(
    const std::vector<std::vector<uint8_t>>& beam_data,
    const ConversionParameters& params,
    const rclcpp::Time& timestamp)
{
    auto msg = std::make_shared<marine_acoustic_msgs::msg::ProjectedSonarImage>();
    msg->image = createSonarImageData(beam_data, marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT8);
    
    return msg;
}

marine_acoustic_msgs::msg::SonarDetections::SharedPtr createSonarDetections(
    const std::vector<std::vector<uint8_t>>& beam_data,
    const ConversionParameters& params,
    const rclcpp::Time& timestamp)
{
    if (beam_data.empty())
    {
        return nullptr;
    }
    
    auto msg = std::make_shared<marine_acoustic_msgs::msg::SonarDetections>();
    
    return msg;
}

marine_acoustic_msgs::msg::SonarImageData createSonarImageData(
    const std::vector<std::vector<uint8_t>>& beam_data,
    uint8_t dtype)
{
    marine_acoustic_msgs::msg::SonarImageData image_data;
    return image_data;
}

} // namespace conversions

NS_FOOT

