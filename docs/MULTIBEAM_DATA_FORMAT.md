# Multibeam Data Format: GLF to marine_acoustic_msgs

## Overview

This document explains how multibeam sonar data flows from the Tritech Gemini SDK (GLF format) to ROS2 `marine_acoustic_msgs` messages.

## GLF Data Structure

### The Raw Format

```cpp
GLF::GMainImage {
    std::vector<UInt8>* m_vecData;          // Intensity values (flattened)
    std::vector<double>* m_vecBearingTable; // Beam angles in DEGREES
    UInt32 m_uiStartRange;                  // First valid sample bin
    UInt32 m_uiEndRange;                    // Last valid sample bin
    // ... other metadata
}
```

### Data Organization

The `m_vecData` is a **1D array representing 2D beam data** in **row-major (beam-major) order**:

```
┌───────────────────────────────────────────────────────┐
│ Beam 0: [s0, s1, s2, ..., sN]                        │
│ Beam 1: [s0, s1, s2, ..., sN]                        │
│ Beam 2: [s0, s1, s2, ..., sN]                        │
│   ...                                                  │
│ Beam M: [s0, s1, s2, ..., sN]                        │
└───────────────────────────────────────────────────────┘
        All stored sequentially in m_vecData
```

**Indexing Formula:**
```cpp
// To access beam B, sample S:
size_t index = B * samples_per_beam + S;
uint8_t intensity = (*m_vecData)[index];
```

### Dimensions

```cpp
size_t num_beams = m_vecBearingTable->size();
size_t total_samples = m_vecData->size();
size_t samples_per_beam = total_samples / num_beams;
```

### Concrete Example: 512 beams × 1500 samples

```
m_vecData->size() = 768,000 bytes (512 × 1500)

Memory layout:
[0-1499]:     Beam 0 samples (leftmost beam)
[1500-2999]:  Beam 1 samples
[3000-4499]:  Beam 2 samples
...
[766500-767999]: Beam 511 samples (rightmost beam)
```

**To get sample 100 from beam 250:**
```cpp
size_t index = 250 * 1500 + 100;  // = 375,100
uint8_t intensity = (*m_vecData)[index];
```

## Beam Angles (Bearing Table)

### Important: Units and Convention

```cpp
std::vector<double>* m_vecBearingTable
```

- **Units**: DEGREES (not radians, despite being `double`)
- **Size**: Exactly `num_beams` entries
- **Order**: Index 0 = first beam angle, Index N = last beam angle
- **Calibration**: Factory-calibrated angles (NOT uniformly spaced!)

### Why Not Uniform?

The bearing table contains **actual beamformed directions** accounting for:
- Array geometry variations
- Phase calibration
- Steering angle corrections
- Hardware imperfections

**Never assume uniform spacing!** Always use the provided angles.

### Example Bearing Table

For a 120° aperture with 5 beams:
```
Index  Angle (deg)   Notes
  0      -60.2       (leftmost, slightly off -60°)
  1      -29.8       
  2        0.1       (center, slightly off 0°)
  3       30.3
  4       59.7       (rightmost)
```

## Converting to marine_acoustic_msgs

### Data Format Compatibility

**Good news:** `marine_acoustic_msgs::SonarImageData` uses the **same row-major ordering** as GLF!

From the [marine_acoustic_msgs documentation](https://docs.ros.org/en/ros2_packages/rolling/api/marine_acoustic_msgs/__README.html):
> "the actual pixel data is in row-major (i.e beam_no major) order"

This means you can use GLF data **directly** in many cases.

### SonarImageData Structure

```cpp
marine_acoustic_msgs::msg::SonarImageData {
    std::vector<uint8_t> data;     // Intensity values (row-major)
    uint8_t dtype;                 // Data type constant
    uint32_t beam_count;           // Number of beams
    uint32_t sample_count;         // Samples per beam
    uint32_t sample0;              // First valid sample index
}
```

### Direct Conversion (No Reorganization)

```cpp
// Option 1: Direct copy (most efficient)
sonar_image_data.data.assign(
    mainImage.m_vecData->begin(),
    mainImage.m_vecData->end()
);
sonar_image_data.beam_count = num_beams;
sonar_image_data.sample_count = samples_per_beam;
sonar_image_data.sample0 = mainImage.m_uiStartRange;
sonar_image_data.dtype = marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT8;
```

