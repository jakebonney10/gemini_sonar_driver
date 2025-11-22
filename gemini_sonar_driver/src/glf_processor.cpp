#include "gemini_sonar_driver/glf_processor.hpp"
#include <cmath>

NS_HEAD

namespace glf_processor
{

PingMetadata extractPingMetadata(
    const GLF::GMainImage& mainImage,
    uint32_t ping_number)
{
    PingMetadata meta;
    
    // Ping identification
    meta.ping_number = ping_number;
    meta.transmit_time_seconds = mainImage.m_dbTxTime;
    
    // Beam configuration
    if (mainImage.m_vecBearingTable) {
        meta.num_beams = mainImage.m_vecBearingTable->size();
    }
    
    if (mainImage.m_vecData && meta.num_beams > 0) {
        meta.samples_per_beam = mainImage.m_vecData->size() / meta.num_beams;
    }
    
    // Range window
    meta.start_range_bin = mainImage.m_uiStartRange;
    meta.end_range_bin = mainImage.m_uiEndRange;
    meta.start_bearing_deg = mainImage.m_uiStartBearing;
    meta.end_bearing_deg = mainImage.m_uiEndBearing;
    
    // Operating parameters
    meta.modulation_frequency_khz = mainImage.m_uiModulationFrequency / 1000.0;
    meta.sound_speed_ms = mainImage.m_fSosAtXd;
    meta.beam_aperture_deg = mainImage.m_fBeamFormAperture;
    meta.gain_percent = mainImage.m_sPercentGain;
    meta.chirp_enabled = mainImage.m_fChirp;
    
    // Data format
    meta.compression_type = mainImage.m_usCompressionType;
    meta.range_compression = mainImage.m_usRangeCompUsed;
    meta.sonar_type = mainImage.m_ucSonarType;
    meta.platform_type = mainImage.m_ucPlatform;
    
    // State flags
    meta.state_flags = mainImage.m_uiStateFlags;
    meta.ping_flags = mainImage.m_usPingFlags;
    
    return meta;
}

BeamData extractBeamData(
    const GLF::GMainImage& mainImage,
    const PingMetadata& metadata)
{
    BeamData beam_data;
    
    // Validate pointers
    if (!mainImage.m_vecData) {
        RCLCPP_ERROR(rclcpp::get_logger("glf_processor"), 
            "GLF data pointer (m_vecData) is null - cannot extract beam data");
        return beam_data;  // Return empty
    }
    
    if (!mainImage.m_vecBearingTable) {
        RCLCPP_ERROR(rclcpp::get_logger("glf_processor"), 
            "GLF bearing table pointer (m_vecBearingTable) is null - cannot extract beam angles");
        return beam_data;  // Return empty
    }
    
    const std::vector<UInt8>& flat_data = *mainImage.m_vecData;
    const std::vector<double>& bearing_table = *mainImage.m_vecBearingTable;
    
    // Validate dimensions match
    const size_t expected_size = metadata.num_beams * metadata.samples_per_beam;
    if (flat_data.size() != expected_size) {
        RCLCPP_ERROR(rclcpp::get_logger("glf_processor"), 
            "Data size mismatch: expected %zu bytes (%u beams × %u samples), got %zu bytes",
            expected_size, metadata.num_beams, metadata.samples_per_beam, flat_data.size());
        return beam_data;  // Data size mismatch
    }
    
    if (bearing_table.size() != metadata.num_beams) {
        RCLCPP_ERROR(rclcpp::get_logger("glf_processor"), 
            "Bearing table size mismatch: expected %u beams, got %zu angles",
            metadata.num_beams, bearing_table.size());
        return beam_data;  // Bearing table size mismatch
    }
    
    // Store raw flat data directly - OPTIMAL for ROS message packing!
    // This is already in row-major (beam-major) format: [beam0_samples, beam1_samples, ...]
    beam_data.flat_data.assign(flat_data.begin(), flat_data.end());
    
    // Extract bearing angles from SDK's factory-calibrated bearing table
    // The bearing table is in RADIANS and contains the actual beam angles
    beam_data.bearing_angles_rad.assign(bearing_table.begin(), bearing_table.end());
    
    // Also create 2D structure for algorithms that need per-beam access
    // (e.g., peak detection, filtering)
    beam_data.beams.resize(metadata.num_beams);
    
    for (size_t beam_idx = 0; beam_idx < metadata.num_beams; ++beam_idx) {
        size_t beam_start = beam_idx * metadata.samples_per_beam;
        size_t beam_end = beam_start + metadata.samples_per_beam;
        
        beam_data.beams[beam_idx].assign(
            flat_data.begin() + beam_start,
            flat_data.begin() + beam_end
        );
    }
    
    return beam_data;
}

bool isCompressed(const GLF::GMainImage& mainImage)
{
    // Check compression type enum
    // ZLIB_COMPRESSION = 0, NO_COMPRESSION = 1, H264_COMPRESSION = 2
    return (mainImage.m_usCompressionType == GLF::GImage::ZLIB_COMPRESSION ||
            mainImage.m_usCompressionType == GLF::GImage::H264_COMPRESSION);
}

bool decompress(GLF::GMainImage& mainImage)
{
    if (!isCompressed(mainImage)) {
        return true;  // No decompression needed
    }
    
    // Check if data pointer is valid
    if (!mainImage.m_vecData) {
        return false;
    }
    
    try {
        // The SDK's UncompressData method handles decompression in-place
        // It uses the m_usCompressionType to determine the algorithm
        std::vector<UInt8> compressed_data = *mainImage.m_vecData;
        std::vector<UInt8> uncompressed_data;
        
        mainImage.UncompressData(
            mainImage.m_usCompressionType,
            compressed_data,
            uncompressed_data
        );
        
        // Replace compressed data with uncompressed
        *mainImage.m_vecData = uncompressed_data;
        
        // Update compression type to indicate no compression
        mainImage.m_usCompressionType = GLF::GImage::NO_COMPRESSION;
        
        return true;
    }
    catch (...) {
        return false;  // Decompression failed
    }
}

double calculateSampleRate(
    const GLF::GMainImage& mainImage,
    const PingMetadata& metadata)
{
    // Sample rate (Hz) = c / (2 * delta_r)
    const double c = metadata.sound_speed_ms;
    const double delta_r = metadata.ping_flags & PingFlags::FREQUENCY_MASK ? FrequencyResolution1200ikd::RES_1200KHZ : FrequencyResolution1200ikd::RES_720KHZ;
    
    double sample_rate_hz = c / (2.0 * delta_r);
    return sample_rate_hz;
}

marine_acoustic_msgs::msg::PingInfo createPingInfo(
    const PingMetadata& metadata)
{
    marine_acoustic_msgs::msg::PingInfo ping_info;
    
    // Center frequency in Hz (convert from kHz)
    double center_frequency_khz = (metadata.ping_flags & PingFlags::FREQUENCY_MASK) ? 720.0 : 1200.0;
    ping_info.frequency = static_cast<float>(center_frequency_khz * 1000.0);

    // Speed of sound in m/s
    ping_info.sound_speed = static_cast<float>(metadata.sound_speed_ms);
    
    // Gemini doesn't report beamwidths in GLF data, leave empty
    ping_info.tx_beamwidths.clear();
    ping_info.rx_beamwidths.clear();
    
    return ping_info;
}

marine_acoustic_msgs::msg::SonarImageData createSonarImageData(
    const BeamData& beam_data,
    const PingMetadata& metadata,
    uint8_t dtype)
{
    marine_acoustic_msgs::msg::SonarImageData sonar_image;
    
    // Set metadata
    sonar_image.dtype = dtype;
    sonar_image.beam_count = metadata.num_beams;
    sonar_image.is_bigendian = false;  // x86_64 is little-endian

    // Direct assignment of flat_data - already in row-major (beam-major) format
    sonar_image.data = beam_data.flat_data;
    
    return sonar_image;
}

marine_acoustic_msgs::msg::RawSonarImage createRawSonarImage(
    const GLF::GMainImage& mainImage,
    const PingMetadata& metadata,
    const BeamData& beam_data,
    const std::string& frame_id)
{
    using marine_acoustic_msgs::msg::RawSonarImage;
    using marine_acoustic_msgs::msg::SonarImageData;

    RawSonarImage msg;

    // Header: transmit time from SDK
    msg.header.frame_id = frame_id;
    msg.header.stamp.sec = static_cast<int32_t>(metadata.transmit_time_seconds);
    msg.header.stamp.nanosec = static_cast<uint32_t>((metadata.transmit_time_seconds - msg.header.stamp.sec) * 1e9);

    // PingInfo (frequency, sound speed)
    msg.ping_info = createPingInfo(metadata);

    // Sample rate in Hz
    msg.sample_rate = static_cast<float>(calculateSampleRate(mainImage, metadata));
    
    // Samples per beam
    msg.samples_per_beam = metadata.samples_per_beam;
    
    // First valid range bin (sample0)
    msg.sample0 = metadata.start_range_bin;

    // Gemini doesn't use per-beam TX steering delays - leave empty
    msg.tx_delays.clear();

    // Gemini doesn't have separate TX steering angles table - leave empty
    msg.tx_angles.clear();

    // RX angles = factory-calibrated bearing table (radians)
    // Direct assignment - bearing_angles_rad is already in the correct format
    msg.rx_angles.assign(
        beam_data.bearing_angles_rad.begin(),
        beam_data.bearing_angles_rad.end()
    );

    // Image payload: row-major, beam-major uint8
    msg.image = createSonarImageData(beam_data, metadata, SonarImageData::DTYPE_UINT8);

    return msg;
}

marine_acoustic_msgs::msg::ProjectedSonarImage createProjectedSonarImage(
    const GLF::GMainImage& mainImage,
    const PingMetadata& metadata,
    const BeamData& beam_data,
    const std::string& frame_id)
{
    using marine_acoustic_msgs::msg::ProjectedSonarImage;
    using marine_acoustic_msgs::msg::SonarImageData;
    using geometry_msgs::msg::Vector3;

    ProjectedSonarImage msg;

    // Header: transmit time from SDK
    msg.header.frame_id = frame_id;
    msg.header.stamp.sec = static_cast<int32_t>(metadata.transmit_time_seconds);
    msg.header.stamp.nanosec = static_cast<uint32_t>((metadata.transmit_time_seconds - msg.header.stamp.sec) * 1e9);

    // PingInfo (frequency, sound speed)
    msg.ping_info = createPingInfo(metadata);

    // Beam directions: unit vectors in sonar frame
    // Convention: Z-forward, X-up, Y-right (consistent with NED for multibeam profilers)
    // For 1D horizontal array: beams lie on Y-Z plane
    // Zero azimuth is along Z-axis (straight out)
    msg.beam_directions.resize(metadata.num_beams);
    for (size_t b = 0; b < metadata.num_beams; ++b) {
        double theta = beam_data.bearing_angles_rad[b]; // azimuth (rotation about X)

        Vector3 v;
        v.x = 0.0;                 // no elevation for 1D horizontal array
        v.y = std::sin(theta);     // right/left (Y-axis)
        v.z = std::cos(theta);     // forward (Z-axis)
        msg.beam_directions[b] = v;
    }

    // Ranges in meters: center of each range bin
    // Computed from range resolution and start bin
    const bool hf = (metadata.ping_flags & PingFlags::FREQUENCY_MASK) != 0;
    const double dr = hf ? FrequencyResolution1200ikd::RES_1200KHZ : FrequencyResolution1200ikd::RES_720KHZ;

    msg.ranges.resize(metadata.samples_per_beam);
    for (size_t r = 0; r < metadata.samples_per_beam; ++r) {
        msg.ranges[r] = static_cast<float>((metadata.start_range_bin + r) * dr);
    }

    // Image payload (same as raw - row-major, beam-major uint8)
    msg.image = createSonarImageData(beam_data, metadata, SonarImageData::DTYPE_UINT8);

    return msg;
}

} // namespace glf_processor

NS_FOOT
