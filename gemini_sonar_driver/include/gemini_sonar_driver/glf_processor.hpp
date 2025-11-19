#pragma once

#include "package_defs.hpp"
#include "gemini_sonar_driver/conversions.hpp"

// Gemini SDK GLF structures
#include "types.h"
#include "GenesisSerializer/GlfLoggerGeminiStructure.h"

// Standard library
#include <vector>
#include <cstdint>

NS_HEAD

/**
 * @brief GLF (Gemini Log Format) data processing utilities
 * 
 * This module extracts sonar data from GLF::GLogTargetImage structures
 * received from the Tritech Gemini SDK and prepares it for conversion
 * to marine_acoustic_msgs formats.
 */
namespace glf_processor
{

//=============================================================================
// Ping Flags Bit Masks (m_usPingFlags)
//=============================================================================

/// Ping flags bit definitions
namespace PingFlags
{
    constexpr uint16_t FREQUENCY_MASK = 0x0001;  ///< Bit 0: Frequency mode
    constexpr uint16_t SOS_MODE_MASK  = 0x8000;  ///< Bit 15: Speed of sound mode
}

/// Ping frequency modes
enum class FrequencyMode : uint8_t
{
    LOW_FREQUENCY  = 0,  ///< 0 = LF mode
    HIGH_FREQUENCY = 1   ///< 1 = HF mode (1200kHz for 1200ik)
};

/// Speed of sound source modes
enum class SoSMode : uint8_t
{
    SONAR_SENSOR = 0,    ///< 0 = Using sonar's internal SOS sensor
    MANUAL       = 1     ///< 1 = Using manually configured SOS value
};

/**
 * @brief Processed ping metadata extracted from GLF::GMainImage
 */
struct PingMetadata
{
    // Ping identification
    uint32_t ping_number = 0;                ///< Sequential ping counter
    double transmit_time_seconds = 0.0;      ///< SDK transmit timestamp (seconds)
    
    // Beam configuration
    uint32_t num_beams = 0;                  ///< Number of beams in this ping
    uint32_t samples_per_beam = 0;           ///< Range bins per beam
    
    // Range window
    uint32_t start_range_bin = 0;            ///< First valid range bin (sample0)
    uint32_t end_range_bin = 0;              ///< Last valid range bin
    uint32_t start_bearing_deg = 0;          ///< Start bearing angle
    uint32_t end_bearing_deg = 0;            ///< End bearing angle
    
    // Operating parameters
    double frequency_khz = 0.0;              ///< Modulation frequency (kHz)
    double sound_speed_ms = 0.0;             ///< Sound speed at transducer (m/s)
    double beam_aperture_deg = 0.0;          ///< Beamforming aperture angle
    int16_t gain_percent = 0;                ///< Receiver gain (%)
    bool chirp_enabled = false;              ///< Chirp mode active
    
    // Data format
    uint16_t compression_type = 0;           ///< Compression scheme (ZLIB, NONE, H264)
    uint16_t range_compression = 0;          ///< Range compression factor
    uint8_t sonar_type = 0;                  ///< Gemini type enum
    uint8_t platform_type = 0;               ///< Specific model (1200ik, 720ik, etc.)
    
    // State flags
    uint32_t state_flags = 0;                ///< Sonar state/orientation flags
    uint16_t ping_flags = 0;                 ///< HF/LF, manual/auto flags
};

/**
 * @brief Processed beam data ready for ROS message conversion
 */
struct BeamData
{
    std::vector<std::vector<uint8_t>> beams; ///< Raw intensity data [beam_index][range_bin]
    std::vector<double> bearing_angles_rad;  ///< Factory-calibrated beam angles (radians)
};

/**
 * @brief Extract ping metadata from GLF::GMainImage
 * 
 * @param mainImage GLF main image structure from SDK
 * @param ping_number Sequential ping counter
 * @return PingMetadata Extracted metadata
 */
PingMetadata extractPingMetadata(
    const GLF::GMainImage& mainImage,
    uint32_t ping_number);

/**
 * @brief Extract beam intensity data from GLF::GMainImage
 * 
 * The SDK provides data in row-major (beam-major) format:
 * [beam0_sample0, beam0_sample1, ..., beam1_sample0, beam1_sample1, ...]
 * 
 * This function reorganizes it into a 2D vector structure for easier processing.
 * 
 * @param mainImage GLF main image structure from SDK
 * @param metadata Pre-extracted ping metadata (for dimensions)
 * @return BeamData Reorganized beam data with bearing angles
 */
BeamData extractBeamData(
    const GLF::GMainImage& mainImage,
    const PingMetadata& metadata);

/**
 * @brief Check if data is compressed and needs decompression
 * 
 * @param mainImage GLF main image structure
 * @return true if compression is active, false otherwise
 */
bool isCompressed(const GLF::GMainImage& mainImage);

/**
 * @brief Decompress GLF image data if needed
 * 
 * @param mainImage GLF main image structure (will be modified if compressed)
 * @return true if decompression succeeded or not needed, false on error
 */
bool decompress(GLF::GMainImage& mainImage);

/**
 * @brief Print diagnostic information for a single ping
 * 
 * Logs detailed information about ping data to verify correct parsing:
 * - Dimensions (beams, samples)
 * - Bearing angles (first 10 beams)
 * - Sample intensity values from multiple positions
 * - Data indexing verification
 * 
 * @param mainImage GLF main image structure from SDK (for SDK bearing table inspection)
 * @param metadata Extracted ping metadata
 * @param beam_data Extracted beam data
 * @param logger ROS2 logger for output
 */
void printPingDiagnostics(
    const GLF::GMainImage& mainImage,
    const PingMetadata& metadata,
    const BeamData& beam_data,
    rclcpp::Logger logger);

} // namespace glf_processor

NS_FOOT
