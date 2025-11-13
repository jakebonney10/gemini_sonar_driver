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

    // Approximate beamwidths using adjacent bearing spacing
    const auto& bearings = params.bearing_table;
    const size_t beam_count = bearings.size();

    if (beam_count > 0)
    {
        ping_info.tx_beamwidths.resize(beam_count, 0.0f);
        ping_info.rx_beamwidths.resize(beam_count, 0.0f);

        if (beam_count == 1)
        {
            // Single beam: width unknown, default to zero
            ping_info.tx_beamwidths[0] = 0.0f;
            ping_info.rx_beamwidths[0] = 0.0f;
        }
        else
        {
            for (size_t i = 0; i < beam_count; ++i)
            {
                const double prev_delta = (i == 0)
                    ? bearings[1] - bearings[0]
                    : bearings[i] - bearings[i - 1];
                const double next_delta = (i + 1 < beam_count)
                    ? bearings[i + 1] - bearings[i]
                    : bearings[i] - bearings[i - 1];

                const double width = 0.5 * (prev_delta + next_delta);
                const float width_abs = static_cast<float>(std::abs(width));
                ping_info.tx_beamwidths[i] = width_abs;
                ping_info.rx_beamwidths[i] = width_abs;
            }
        }
    }
    else
    {
        ping_info.tx_beamwidths.clear();
        ping_info.rx_beamwidths.clear();
    }
    
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
    double sample_rate = params.sample_rate_hz;
    if (sample_rate <= 0.0 && params.range_m > 0.0 && params.bins_per_beam > 0)
    {
        sample_rate = (static_cast<double>(params.bins_per_beam) * params.sound_speed_ms) /
            (2.0 * params.range_m);
    }
    msg->sample_rate = static_cast<float>(sample_rate);
    
    // Samples per beam
    msg->samples_per_beam = params.bins_per_beam;
    
    // First sample (typically 0 for starting at sonar head)
    msg->sample0 = params.start_sample;
    
    size_t num_beams = beam_data.size();
    
    // TX delays (zero for single-transmit sonar)
    msg->tx_delays.assign(num_beams, 0.0f);
    
    // TX angles (along-track, typically 0 for FLS)
    msg->tx_angles.assign(num_beams, 0.0f);
    
    // RX angles (across-track)
    msg->rx_angles.resize(num_beams);
    for (size_t i = 0; i < num_beams; ++i)
    {
        msg->rx_angles[i] = getBeamAngle(i, params);
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
        float angle_rad = getBeamAngle(i, params);
        
    // Beam direction as unit vector in Y-Z plane:
    //   Z: forward (look direction)
    //   Y: starboard (positive to starboard)
    //   X: up (zero for planar fan)
    msg->beam_directions[i].x = 0.0;
    msg->beam_directions[i].y = std::sin(angle_rad);
    msg->beam_directions[i].z = std::cos(angle_rad);
    }
    
    // Set range bins (center of each bin in meters)
    msg->ranges.resize(params.bins_per_beam);
    const double bin_resolution = (params.bin_resolution_m > 0.0)
        ? params.bin_resolution_m
        : ((params.bins_per_beam > 0)
            ? (params.range_m / static_cast<double>(params.bins_per_beam))
            : 0.0);
    const double start_sample = static_cast<double>(params.start_sample);

    for (int i = 0; i < params.bins_per_beam; ++i)
    {
        const double sample_center = start_sample + static_cast<double>(i) + 0.5;
        msg->ranges[i] = static_cast<float>(sample_center * bin_resolution);
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
        size_t peak_index = 0;
        float peak_intensity = 0.0f;
        const float detection_range = extractDetectionRange(
            beam_data[i], params, &peak_index, &peak_intensity);

        const float sound_speed = static_cast<float>(params.sound_speed_ms);
        msg->two_way_travel_times[i] = (sound_speed > 0.0f)
            ? (2.0f * detection_range / sound_speed)
            : 0.0f;

        msg->intensities[i] = peak_intensity;
    }
    
    // TX delays (zero for single-sector sonar)
    msg->tx_delays.assign(num_beams, 0.0f);
    
    // Set beam angles
    msg->tx_angles.assign(num_beams, 0.0f);  // Along-track (forward/aft) - zero for FLS
    msg->rx_angles.resize(num_beams);
    
    for (size_t i = 0; i < num_beams; ++i)
    {
        msg->rx_angles[i] = getBeamAngle(i, params);
    }
    
    return msg;
}

float extractDetectionRange(
    const std::vector<uint8_t>& beam_samples,
    const ConversionParameters& params,
    size_t* peak_index,
    float* peak_intensity)
{
    if (beam_samples.empty())
    {
        if (peak_index)
        {
            *peak_index = 0;
        }
        if (peak_intensity)
        {
            *peak_intensity = 0.0f;
        }
        return 0.0f;
    }

    const auto max_it = std::max_element(beam_samples.begin(), beam_samples.end());
    const size_t max_index = static_cast<size_t>(std::distance(beam_samples.begin(), max_it));

    if (peak_index)
    {
        *peak_index = max_index;
    }
    if (peak_intensity)
    {
        *peak_intensity = static_cast<float>(*max_it);
    }

    const double bin_resolution = (params.bin_resolution_m > 0.0)
        ? params.bin_resolution_m
        : ((params.bins_per_beam > 0)
            ? (params.range_m / static_cast<double>(params.bins_per_beam))
            : 0.0);

    const double sample_center = static_cast<double>(params.start_sample) +
        static_cast<double>(max_index) + 0.5;

    return static_cast<float>(sample_center * bin_resolution);
}

float getBeamAngle(size_t beam_index, const ConversionParameters& params)
{
    // Bearing table is stored in radians after preprocessing in the node
    return static_cast<float>(params.bearing_table[beam_index]);
}

marine_acoustic_msgs::msg::SonarImageData createSonarImageData(
    const std::vector<std::vector<uint8_t>>& beam_data,
    uint8_t dtype)
{
    marine_acoustic_msgs::msg::SonarImageData image_data;
    
    image_data.dtype = dtype;
    image_data.beam_count = beam_data.size();
    image_data.is_bigendian = false;
    
    if (beam_data.empty())
    {
        return image_data;
    }
    
    // Flatten 2D beam data into 1D array in ROW-MAJOR order (sample-by-sample)
    // acoustic_msgs_tools expects: index = sample_idx * num_beams + beam_idx
    // Data layout: [beam0_sample0, beam1_sample0, ..., beamN_sample0, beam0_sample1, beam1_sample1, ...]
    size_t num_beams = beam_data.size();
    size_t samples_per_beam = beam_data[0].size();
    
    image_data.data.reserve(num_beams * samples_per_beam);
    
    // Iterate sample-by-sample (row-by-row), collecting all beams at each sample
    for (size_t sample_idx = 0; sample_idx < samples_per_beam; ++sample_idx)
    {
        for (size_t beam_idx = 0; beam_idx < num_beams; ++beam_idx)
        {
            image_data.data.push_back(beam_data[beam_idx][sample_idx]);
        }
    }
    
    return image_data;
}

} // namespace conversions

NS_FOOT

