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

} // namespace glf_processor

NS_FOOT
