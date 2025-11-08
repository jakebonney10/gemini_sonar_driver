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
    
    // For now, we don't have per-beam beamwidth info from SDK
    // These would need to be populated from actual Gemini head data
    ping_info.tx_beamwidths.clear();
    ping_info.rx_beamwidths.clear();
    
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
    
    // Sample rate (samples per second)
    // This is the rate at which range samples are acquired
    float sample_rate = (params.bins_per_beam * params.sound_speed_ms) / (2.0 * params.range_m);
    msg->sample_rate = sample_rate;
    
    // Samples per beam
    msg->samples_per_beam = params.bins_per_beam;
    
    // First sample (typically 0 for starting at sonar head)
    msg->sample0 = 0.0;
    
    size_t num_beams = beam_data.size();
    
    // TX delays (zero for single-transmit sonar)
    msg->tx_delays.assign(num_beams, 0.0f);
    
    // TX angles (along-track, typically 0 for FLS)
    msg->tx_angles.assign(num_beams, 0.0f);
    
    // RX angles (across-track)
    msg->rx_angles.resize(num_beams);
    for (size_t i = 0; i < num_beams; ++i)
    {
        msg->rx_angles[i] = calculateBeamAngle(i, num_beams, params.beam_spacing_deg);
    }
    
    // Image data
    msg->image = createSonarImageData(beam_data, marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT8);
    
    return msg;
}

marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr createProjectedSonarImage(
    const std::vector<std::vector<uint8_t>>& beam_data,
    const ConversionParameters& params,
    const rclcpp::Time& timestamp)
{
    if (beam_data.empty())
    {
        return nullptr;
    }
    
    auto msg = std::make_shared<marine_acoustic_msgs::msg::ProjectedSonarImage>();
    
    // Header
    msg->header.stamp = timestamp;
    msg->header.frame_id = params.frame_id;
    
    // Ping info
    msg->ping_info = createPingInfo(params);
    
    size_t num_beams = beam_data.size();
    
    // Set beam directions as 3D unit vectors
    msg->beam_directions.resize(num_beams);
    for (size_t i = 0; i < num_beams; ++i)
    {
        float angle_rad = calculateBeamAngle(i, num_beams, params.beam_spacing_deg);
        
        // Beam direction as unit vector (assuming forward sonar looking in +Z direction)
        // X: forward (sonar look direction)
        // Y: across-track (positive = starboard)
        // Z: vertical (positive = up)
        msg->beam_directions[i].x = std::cos(angle_rad);  // Forward component
        msg->beam_directions[i].y = std::sin(angle_rad);  // Across-track component
        msg->beam_directions[i].z = 0.0;                  // Assuming level sonar
    }
    
    // Set range bins (center of each bin in meters)
    msg->ranges.resize(params.bins_per_beam);
    for (int i = 0; i < params.bins_per_beam; ++i)
    {
        msg->ranges[i] = ((static_cast<float>(i) + 0.5f) / params.bins_per_beam) * params.range_m;
    }
    
    // Image data
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
    
    // Header
    msg->header.stamp = timestamp;
    msg->header.frame_id = params.frame_id;
    
    // Ping info
    msg->ping_info = createPingInfo(params);
    
    size_t num_beams = beam_data.size();
    
    // Initialize detection flags (all good by default)
    msg->flags.resize(num_beams);
    for (size_t i = 0; i < num_beams; ++i)
    {
        msg->flags[i].flag = marine_acoustic_msgs::msg::DetectionFlag::DETECT_OK;
        // TODO: Add actual beam quality checking from SDK data
    }
    
    // Set two-way travel times and intensities for each beam
    msg->two_way_travel_times.resize(num_beams);
    msg->intensities.resize(num_beams);
    
    for (size_t i = 0; i < num_beams; ++i)
    {
        // Extract detection range from beam data
        float detection_range = extractDetectionRange(beam_data[i], params.range_m);
        msg->two_way_travel_times[i] = (2.0f * detection_range) / params.sound_speed_ms;
        
        // Extract intensity
        msg->intensities[i] = extractIntensity(beam_data[i]);
    }
    
    // TX delays (zero for single-sector sonar)
    msg->tx_delays.assign(num_beams, 0.0f);
    
    // Set beam angles
    msg->tx_angles.assign(num_beams, 0.0f);  // Along-track (forward/aft) - zero for FLS
    msg->rx_angles.resize(num_beams);
    
    for (size_t i = 0; i < num_beams; ++i)
    {
        msg->rx_angles[i] = calculateBeamAngle(i, num_beams, params.beam_spacing_deg);
    }
    
    return msg;
}

float extractDetectionRange(const std::vector<uint8_t>& beam_samples, float max_range_m)
{
    if (beam_samples.empty())
    {
        return max_range_m;
    }
    
    // Find the sample with maximum intensity
    auto max_it = std::max_element(beam_samples.begin(), beam_samples.end());
    size_t max_index = std::distance(beam_samples.begin(), max_it);
    
    // Convert sample index to range
    float range = (static_cast<float>(max_index) / beam_samples.size()) * max_range_m;
    
    return range;
}

float extractIntensity(const std::vector<uint8_t>& beam_samples)
{
    if (beam_samples.empty())
    {
        return 0.0f;
    }
    
    // Return maximum intensity value
    return static_cast<float>(*std::max_element(beam_samples.begin(), beam_samples.end()));
}

float calculateBeamAngle(size_t beam_index, size_t num_beams, double beam_spacing_deg)
{
    // Calculate angle with center beam at 0
    // Positive angles to starboard, negative to port
    double angle_deg = (static_cast<double>(beam_index) - num_beams / 2.0) * beam_spacing_deg;
    
    // Convert to radians
    return static_cast<float>(angle_deg * M_PI / 180.0);
}

marine_acoustic_msgs::msg::SonarImageData createSonarImageData(
    const std::vector<std::vector<uint8_t>>& beam_data,
    uint8_t dtype)
{
    marine_acoustic_msgs::msg::SonarImageData image_data;
    
    image_data.dtype = dtype;
    image_data.beam_count = beam_data.size();
    image_data.is_bigendian = false;
    
    // Flatten 2D beam data into 1D array
    // Data is organized as [beam0_sample0, beam0_sample1, ..., beam1_sample0, ...]
    image_data.data.clear();
    for (const auto& beam : beam_data)
    {
        image_data.data.insert(image_data.data.end(), beam.begin(), beam.end());
    }
    
    return image_data;
}

} // namespace conversions

NS_FOOT

