# GLF (Gemini Log Format) Data Parsing

## Overview

When using the **Svs5Sequencer API**, sonar data is delivered in **GLF (Gemini Log Format)** as processed, beamformed images. This is different from the low-level GeminiComms API which provides raw `CGemPingHead`, `CGemPingLine`, and `CGemPingTail` messages.

## GLF Structure Hierarchy

```
GLF::GLogTargetImage
├── console::PublicMessageHeader m_header
│   └── CommonInterfaceHeader m_ciHeader (21 bytes)
├── GLF::GMainImage m_mainImage
│   ├── [GImage base class members]
│   │   ├── UInt16 m_usImageVersion
│   │   ├── UInt16 m_usRangeCompUsed (range compression factor)
│   │   ├── UInt16 m_usCompressionType (ZLIB_COMPRESSION, NO_COMPRESSION, H264_COMPRESSION)
│   │   ├── UInt32 m_uiStartBearing
│   │   ├── UInt32 m_uiEndBearing
│   │   ├── UInt32 m_uiStartRange
│   │   ├── UInt32 m_uiEndRange
│   │   └── std::vector<UInt8>* m_vecData (actual image data)
│   ├── std::vector<double>* m_vecBearingTable (beam angles in radians)
│   ├── UInt32 m_uiStateFlags
│   │   - Bit 15-13: Sonar orientation (0=up, 1=down)
│   │   - Bit 7: Vertical beam mode (0=wide, 1=narrow)
│   ├── UInt32 m_uiModulationFrequency (operating frequency in Hz)
│   ├── float m_fBeamFormAperture
│   ├── double m_dbTxTime (transmit timestamp)
│   ├── UInt16 m_usPingFlags
│   │   - Bit 0: Frequency (1=HF, 0=LF)
│   │   - Bit 15: SOS mode (1=manual, 0=sonar)
│   ├── float m_fSosAtXd (speed of sound at transducer)
│   ├── Int16 m_sPercentGain (receiver gain percentage)
│   ├── bool m_fChirp (chirp enabled/disabled)
│   ├── UInt8 m_ucSonarType (eGeminiMk1Imager, eGeminiMk1Profiler, eGeminiMk2Imager)
│   └── UInt8 m_ucPlatform (eSonarTypeNone, eGemini720is, eGemini720ik, eGemini720im, eGemini1200ik, etc.)
└── GLF::GZoomImage m_aczImage (acoustic zoom - if active)
    ├── bool m_fActive
    ├── UInt16 m_usId
    ├── double m_dMagnitude
    └── [GImage base class members for zoom data]
```

## Data Organization

### Image Data Layout

The `m_vecData` contains **flattened 2D beam data**:

```
[beam0_sample0, beam0_sample1, ..., beam0_sampleN,
 beam1_sample0, beam1_sample1, ..., beam1_sampleN,
 ...
 beamM_sample0, beamM_sample1, ..., beamM_sampleN]
```

**Dimensions:**
- `num_beams` = `m_vecBearingTable->size()`
- `total_samples` = `m_vecData->size()`
- `samples_per_beam` = `total_samples / num_beams`

### Bearing Table

The `m_vecBearingTable` contains **precise beam angles in radians** from the sonar's beamformer. These are the actual formed beam directions, not calculated approximations.

- **Indexing**: `bearing_angle = (*m_vecBearingTable)[beam_index]`
- **Units**: Radians
- **Convention**: Depends on sonar orientation (check `m_uiStateFlags`)

## Parsing GLF Data in ROS2

### 1. Receive GLF Message

```cpp
void GeminiSonarNode::processSvs5Message(unsigned int messageType, unsigned int size, const char* const value)
{
    switch (static_cast<SequencerApi::ESvs5MessageType>(messageType))
    {
        case SequencerApi::GLF_LIVE_TARGET_IMAGE:
        {
            // Cast to GLF structure
            GLF::GLogTargetImage* image = (GLF::GLogTargetImage*)value;
            if (image) {
                processGLFImage(*image);
            }
            break;
        }
    }
}
```

### 2. Extract Image Data

```cpp
void GeminiSonarNode::processGLFImage(const GLF::GLogTargetImage& image)
{
    const GLF::GMainImage& mainImage = image.m_mainImage;
    
    // Get data pointers
    const std::vector<UInt8>* imageData = mainImage.m_vecData;
    const std::vector<double>* bearingTable = mainImage.m_vecBearingTable;
    
    // Validate data
    if (!imageData || !bearingTable || imageData->empty() || bearingTable->empty()) {
        return;  // No data
    }
    
    // Calculate dimensions
    size_t num_beams = bearingTable->size();
    size_t total_samples = imageData->size();
    size_t samples_per_beam = total_samples / num_beams;
}
```

### 3. Convert to Beam Structure

```cpp
// Convert flattened data to 2D beam structure
std::vector<std::vector<uint8_t>> beam_data;
beam_data.reserve(num_beams);

for (size_t beam_idx = 0; beam_idx < num_beams; ++beam_idx)
{
    size_t start_idx = beam_idx * samples_per_beam;
    size_t end_idx = start_idx + samples_per_beam;
    
    if (end_idx <= imageData->size())
    {
        std::vector<uint8_t> beam_samples(
            imageData->begin() + start_idx,
            imageData->begin() + end_idx
        );
        beam_data.push_back(std::move(beam_samples));
    }
}
```

### 4. Extract Metadata

```cpp
// Update conversion parameters from GLF metadata
conversion_params_.num_beams = num_beams;
conversion_params_.bins_per_beam = samples_per_beam;
conversion_params_.frequency_khz = mainImage.m_uiModulationFrequency / 1000.0;
conversion_params_.sound_speed_ms = mainImage.m_fSosAtXd;

// Calculate range from start/end range fields
float range_bins = mainImage.m_uiEndRange - mainImage.m_uiStartRange;
conversion_params_.range_m = (range_bins * conversion_params_.sound_speed_ms) / 
                              (2.0 * conversion_params_.frequency_khz * 1000.0);

// Get gain setting
int gain = mainImage.m_sPercentGain;

// Get chirp status
bool chirp_enabled = mainImage.m_fChirp;

// Get transmit time
double tx_time = mainImage.m_dbTxTime;
```

### 5. Publish ROS2 Messages

```cpp
// Use the conversions module to create marine_acoustic_msgs
auto raw_msg = conversions::createRawSonarImage(beam_data, conversion_params_, timestamp);
if (raw_msg) {
    publishers_.raw_sonar_image_->publish(*raw_msg);
}

auto detections_msg = conversions::createSonarDetections(beam_data, conversion_params_, timestamp);
if (detections_msg) {
    publishers_.sonar_detections_->publish(*detections_msg);
}

auto proj_msg = conversions::createProjectedSonarImage(beam_data, conversion_params_, timestamp);
if (proj_msg) {
    publishers_.projected_sonar_image_->publish(*proj_msg);
}
```

## Platform Detection

Use the `m_ucPlatform` field to identify the specific sonar model:

```cpp
enum ESonarType
{
    eSonarTypeNone             = 0,
    eGemini720is               = 1,
    eGemini720ik               = 2,
    eGemini720im               = 3,    // Micron Gemini
    eGemini1200ik              = 4,    // Your sonar!
    eGemini720ik360Sector1     = 5,
    eGemini720ik360Sector2     = 6,
    eGemini720ik360Sector3     = 7,
    eGemini1200nbik            = 8,
    eGemini1200id              = 9,
    eMicronGemini1200d         = 10
};

// In your code:
if (mainImage.m_ucPlatform == GLF::GMainImage::eGemini1200ik) {
    // 1200ik-specific processing
}
```

## Sonar Type Detection

The `m_ucSonarType` indicates the beamforming algorithm:

```cpp
enum EGeminiType
{
    eGeminiMk1Imager    = 0,  // Spreading algorithm
    eGeminiMk1Profiler  = 1,  // Software gain
    eGeminiMk2Imager    = 2   // BMG rescale
};

// Your 1200ik will be MK2:
if (mainImage.m_ucSonarType == GLF::GMainImage::eGeminiMk2Imager) {
    // MK2-specific processing
}
```

## Compression Handling

GLF data can be compressed. Check `m_usCompressionType`:

```cpp
enum ECompressionType {
    ZLIB_COMPRESSION,   // zLib compression (default)
    NO_COMPRESSION,     // Raw data
    H264_COMPRESSION    // H.264 video compression
};

// If compressed, use:
// mainImage.UncompressData(compressionType, inData, outData);
```

## Acoustic Zoom

If acoustic zoom is enabled (`m_aczImage.m_fActive`), you'll have two image data sets:

```cpp
if (image.m_aczImage.m_fActive) {
    // Process zoomed region
    const GLF::GZoomImage& zoomImage = image.m_aczImage;
    double magnification = zoomImage.m_dMagnitude;
    
    // Zoom image uses the same bearing table as main image
    // but has different range/bearing extents
}
```

## References

- **GLF Structure Definition**: `GenesisSerializer/GlfLoggerGeminiStructure.h`
- **Type Definitions**: `types.h` (UInt8, UInt16, UInt32, etc.)
- **Example Usage**: `GeminiSDK/src/GeminiSDKGuiApp/gemininetwork.cpp`
- **Svs5 API**: `Svs5Seq/Svs5SequencerApi.h`
