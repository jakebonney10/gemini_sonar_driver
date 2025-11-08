#pragma once

// Local package includes
#include "package_defs.hpp"

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <marine_acoustic_msgs/msg/raw_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/sonar_detections.hpp>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>
#include <marine_acoustic_msgs/msg/ping_info.hpp>
#include <marine_acoustic_msgs/msg/detection_flag.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// Standard library
#include <vector>
#include <cstdint>
#include <memory>
#include <string>

NS_HEAD

/**
 * @brief Conversion utilities for Gemini sonar data to ROS2 messages
 * 
 * This namespace contains functions to convert Gemini SDK data structures
 * into marine_acoustic_msgs format for ROS2 publishing.
 */
namespace conversions
{

/**
 * @brief Parameters needed for sonar data conversion
 */
struct ConversionParameters
{
    // Sonar configuration
    double frequency_khz = 720.0;        ///< Operating frequency in kHz
    double sound_speed_ms = 1500.0;      ///< Sound speed in m/s
    double range_m = 75.0;               ///< Maximum range in meters
    int num_beams = 512;                 ///< Number of beams
    int bins_per_beam = 1500;            ///< Range cells per beam
    double beam_spacing_deg = 0.25;      ///< Beam spacing in degrees
    
    // Frame information
    std::string frame_id = "gemini";     ///< TF frame ID
};

/**
 * @brief Create PingInfo message from conversion parameters
 * 
 * @param params Conversion parameters
 * @return marine_acoustic_msgs::msg::PingInfo Filled ping info message
 */
marine_acoustic_msgs::msg::PingInfo createPingInfo(
    const ConversionParameters& params);

/**
 * @brief Convert beam data to RawSonarImage message
 * 
 * RawSonarImage contains the raw beam data with beam angles and timing information.
 * This is the most complete representation of the sonar data.
 * 
 * @param beam_data Vector of beam data (each beam is a vector of samples)
 * @param params Conversion parameters
 * @param timestamp ROS timestamp for the message
 * @return Shared pointer to RawSonarImage message
 */
marine_acoustic_msgs::msg::RawSonarImage::SharedPtr createRawSonarImage(
    const std::vector<std::vector<uint8_t>>& beam_data,
    const ConversionParameters& params,
    const rclcpp::Time& timestamp);

/**
 * @brief Convert beam data to ProjectedSonarImage message
 * 
 * ProjectedSonarImage represents the sonar data in a cartesian coordinate system
 * with explicit beam directions and range bins.
 * 
 * @param beam_data Vector of beam data (each beam is a vector of samples)
 * @param params Conversion parameters
 * @param timestamp ROS timestamp for the message
 * @return Shared pointer to ProjectedSonarImage message
 */
marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr createProjectedSonarImage(
    const std::vector<std::vector<uint8_t>>& beam_data,
    const ConversionParameters& params,
    const rclcpp::Time& timestamp);

/**
 * @brief Convert beam data to SonarDetections message
 * 
 * SonarDetections provides the multibeam detection information including
 * travel times, angles, and intensities. This is the recommended format
 * for forward-looking sonar (FLS) applications.
 * 
 * @param beam_data Vector of beam data (each beam is a vector of samples)
 * @param params Conversion parameters
 * @param timestamp ROS timestamp for the message
 * @return Shared pointer to SonarDetections message
 */
marine_acoustic_msgs::msg::SonarDetections::SharedPtr createSonarDetections(
    const std::vector<std::vector<uint8_t>>& beam_data,
    const ConversionParameters& params,
    const rclcpp::Time& timestamp);

/**
 * @brief Extract detection range from beam data
 * 
 * Finds the range of the strongest return in a beam by finding the maximum
 * intensity sample.
 * 
 * @param beam_samples Vector of intensity samples for one beam
 * @param max_range_m Maximum range in meters
 * @return Detection range in meters
 */
float extractDetectionRange(
    const std::vector<uint8_t>& beam_samples,
    float max_range_m);

/**
 * @brief Extract intensity from beam data
 * 
 * Returns the maximum intensity value in the beam.
 * 
 * @param beam_samples Vector of intensity samples for one beam
 * @return Maximum intensity value (0-255 for uint8)
 */
float extractIntensity(
    const std::vector<uint8_t>& beam_samples);

/**
 * @brief Calculate beam angle for a given beam index
 * 
 * Calculates the across-track angle for a beam based on the beam index
 * and beam spacing. Center beam is at 0 degrees, positive to starboard,
 * negative to port.
 * 
 * @param beam_index Index of the beam (0 to num_beams-1)
 * @param num_beams Total number of beams
 * @param beam_spacing_deg Spacing between beams in degrees
 * @return Beam angle in radians
 */
float calculateBeamAngle(
    size_t beam_index,
    size_t num_beams,
    double beam_spacing_deg);

/**
 * @brief Create SonarImageData structure from beam data
 * 
 * Flattens the 2D beam data into a 1D array suitable for SonarImageData.
 * 
 * @param beam_data Vector of beam data (each beam is a vector of samples)
 * @param dtype Data type constant (DTYPE_UINT8, etc.)
 * @return SonarImageData message
 */
marine_acoustic_msgs::msg::SonarImageData createSonarImageData(
    const std::vector<std::vector<uint8_t>>& beam_data,
    uint8_t dtype);

} // namespace conversions

NS_FOOT

