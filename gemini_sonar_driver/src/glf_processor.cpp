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
    meta.frequency_khz = mainImage.m_uiModulationFrequency / 1000.0;
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
    if (!mainImage.m_vecData || !mainImage.m_vecBearingTable) {
        return beam_data;  // Return empty
    }
    
    const std::vector<UInt8>& flat_data = *mainImage.m_vecData;
    const std::vector<double>& bearing_table = *mainImage.m_vecBearingTable;
    
    // Validate dimensions match
    if (flat_data.size() != metadata.num_beams * metadata.samples_per_beam) {
        return beam_data;  // Data size mismatch
    }
    
    // Extract bearing angles from SDK's factory-calibrated bearing table
    // The bearing table is in RADIANS and contains the actual beam angles
    // from factory calibration (not offsets!)
    beam_data.bearing_angles_rad.reserve(bearing_table.size());
    for (double angle_rad : bearing_table) {
        beam_data.bearing_angles_rad.push_back(angle_rad);
    }
    
    // Reorganize flat data into 2D beam structure for easier manipulation
    // GLF format: [beam0_all_samples, beam1_all_samples, ...]
    // We convert to: beams[beam_index][sample_index]
    beam_data.beams.resize(metadata.num_beams);
    
    for (size_t beam_idx = 0; beam_idx < metadata.num_beams; ++beam_idx) {
        // Calculate starting position for this beam in the flat array
        size_t beam_start = beam_idx * metadata.samples_per_beam;
        size_t beam_end = beam_start + metadata.samples_per_beam;
        
        // Extract this beam's samples
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

void printPingDiagnostics(
    const GLF::GMainImage& mainImage,
    const PingMetadata& metadata,
    const BeamData& beam_data,
    rclcpp::Logger logger)
{
    RCLCPP_INFO(logger, "=== PING DIAGNOSTICS ===");
    RCLCPP_INFO(logger, "Ping #%u | Tx Time: %.6f s", 
                metadata.ping_number, metadata.transmit_time_seconds);
    
    // Dimensions
    RCLCPP_INFO(logger, "Dimensions: %u beams × %u samples/beam = %zu total samples",
                metadata.num_beams, metadata.samples_per_beam,
                metadata.num_beams * metadata.samples_per_beam);
    
    // Operating parameters
    RCLCPP_INFO(logger, "Frequency: %.1f kHz | SOS: %.1f m/s | Gain: %d%% | Chirp: %s",
                metadata.frequency_khz, metadata.sound_speed_ms,
                metadata.gain_percent, metadata.chirp_enabled ? "ON" : "OFF");
    
    // Range window
    RCLCPP_INFO(logger, "Range bins: [%u, %u] | Bearing: [%u, %u]",
                metadata.start_range_bin, metadata.end_range_bin,
                metadata.start_bearing_deg, metadata.end_bearing_deg);
    
    // Bearing angles (first 10 beams) - show actual values with precision
    if (beam_data.bearing_angles_rad.size() >= 10) {
        RCLCPP_INFO(logger, "First 10 bearing angles (from factory calibration):");
        for (size_t i = 0; i < 10; ++i) {
            double angle_deg = beam_data.bearing_angles_rad[i] * 180.0 / M_PI;
            RCLCPP_INFO(logger, "  Beam[%zu]: %.3f°", i, angle_deg);
        }
        
        // Show angular spacing between first two beams
        if (beam_data.bearing_angles_rad.size() >= 2) {
            double spacing = (beam_data.bearing_angles_rad[1] - beam_data.bearing_angles_rad[0]) * 180.0 / M_PI;
            RCLCPP_INFO(logger, "Angular spacing (beam 0->1): %.3f°", spacing);
        }
    }
    
    // Show raw SDK bearing table values to confirm they're in radians
    if (mainImage.m_vecBearingTable && mainImage.m_vecBearingTable->size() >= 3) {
        const std::vector<double>& sdk_bearing_table = *mainImage.m_vecBearingTable;
        RCLCPP_INFO(logger, "SDK bearing table (radians): %.6f, %.6f, %.6f",
                    sdk_bearing_table[0], sdk_bearing_table[1], sdk_bearing_table[2]);
        RCLCPP_INFO(logger, "  = %.3f°, %.3f°, %.3f°",
                    sdk_bearing_table[0] * 180.0 / M_PI,
                    sdk_bearing_table[1] * 180.0 / M_PI,
                    sdk_bearing_table[2] * 180.0 / M_PI);
    }
    
    // Verify data extraction by sampling multiple positions
    if (!beam_data.beams.empty() && metadata.samples_per_beam > 0) {
        RCLCPP_INFO(logger, "Sample intensity values (to verify indexing):");
        
        // Count non-zero samples for data quality check
        size_t total_samples_checked = 0;
        size_t non_zero_count = 0;
        
        // Sample from first beam
        size_t beam_idx = 0;
        if (beam_idx < beam_data.beams.size()) {
            uint8_t near = beam_data.beams[beam_idx][0];
            uint8_t mid = beam_data.beams[beam_idx][metadata.samples_per_beam / 2];
            uint8_t far = beam_data.beams[beam_idx][metadata.samples_per_beam - 1];
            
            RCLCPP_INFO(logger, "  Beam 0: near[0]=%u mid[%u]=%u far[%u]=%u",
                        near, metadata.samples_per_beam / 2, mid,
                        metadata.samples_per_beam - 1, far);
            
            total_samples_checked += 3;
            if (near > 0) non_zero_count++;
            if (mid > 0) non_zero_count++;
            if (far > 0) non_zero_count++;
        }
        
        // Sample from middle beam
        beam_idx = metadata.num_beams / 2;
        if (beam_idx < beam_data.beams.size()) {
            uint8_t near = beam_data.beams[beam_idx][0];
            uint8_t mid = beam_data.beams[beam_idx][metadata.samples_per_beam / 2];
            uint8_t far = beam_data.beams[beam_idx][metadata.samples_per_beam - 1];
            
            RCLCPP_INFO(logger, "  Beam %u (middle): near[0]=%u mid[%u]=%u far[%u]=%u",
                        metadata.num_beams / 2, near,
                        metadata.samples_per_beam / 2, mid,
                        metadata.samples_per_beam - 1, far);
            
            total_samples_checked += 3;
            if (near > 0) non_zero_count++;
            if (mid > 0) non_zero_count++;
            if (far > 0) non_zero_count++;
        }
        
        // Sample from last beam
        beam_idx = metadata.num_beams - 1;
        if (beam_idx < beam_data.beams.size()) {
            uint8_t near = beam_data.beams[beam_idx][0];
            uint8_t mid = beam_data.beams[beam_idx][metadata.samples_per_beam / 2];
            uint8_t far = beam_data.beams[beam_idx][metadata.samples_per_beam - 1];
            
            RCLCPP_INFO(logger, "  Beam %u (last): near[0]=%u mid[%u]=%u far[%u]=%u",
                        metadata.num_beams - 1, near,
                        metadata.samples_per_beam / 2, mid,
                        metadata.samples_per_beam - 1, far);
            
            total_samples_checked += 3;
            if (near > 0) non_zero_count++;
            if (mid > 0) non_zero_count++;
            if (far > 0) non_zero_count++;
        }
        
        // Data quality warning
        if (non_zero_count == 0) {
            RCLCPP_WARN(logger, "⚠️  ALL sampled intensity values are ZERO - sonar likely out of water or no targets");
        } else {
            RCLCPP_INFO(logger, "Data quality: %zu/%zu samples non-zero (%.1f%%)",
                        non_zero_count, total_samples_checked,
                        100.0 * non_zero_count / total_samples_checked);
        }
    }
    
    RCLCPP_INFO(logger, "========================");
}

} // namespace glf_processor

NS_FOOT
