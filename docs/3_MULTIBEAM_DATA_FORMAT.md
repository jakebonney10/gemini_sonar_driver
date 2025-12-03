# Multibeam Data Format: GLF to marine_acoustic_msgs

## Overview

This document explains how multibeam sonar data flows from the Gemini SDK (GLF format) to ROS2 `marine_acoustic_msgs` messages.

## GLF Data Structure

```cpp
GLF::GMainImage {
    std::vector<UInt8>* m_vecData;          // Intensity values
    std::vector<double>* m_vecBearingTable; // Beam angles (radians)
    UInt32 m_uiStartRange;                  // First valid sample
    UInt32 m_uiEndRange;                    // Last valid sample
}
```

### Data Organization

GLF stores data in **beam-major order**: all samples for beam 0, then beam 1, etc.

**Indexing:**
```cpp
// Access beam B, sample S:
index = B * samples_per_beam + S;
intensity = (*m_vecData)[index];
```

**Dimensions:**
```cpp
num_beams = m_vecBearingTable->size();
samples_per_beam = m_vecData->size() / num_beams;
```

## Converting to marine_acoustic_msgs

`marine_acoustic_msgs::msg::RawSonarImage` uses the **same beam-major ordering** as GLF, allowing direct data copy.

### Conversion Example

```cpp
// Create RawSonarImage message
auto msg = marine_acoustic_msgs::msg::RawSonarImage();

// Copy intensity data directly
msg.image.data.assign(
    mainImage.m_vecData->begin(),
    mainImage.m_vecData->end()
);

// Set dimensions
msg.image.beam_count = num_beams;
msg.image.sample_count = samples_per_beam;
msg.image.sample0 = mainImage.m_uiStartRange;
msg.image.dtype = marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT8;

// Copy beam angles (convert radians to degrees if needed)
msg.beam_directions.resize(num_beams);
for (size_t i = 0; i < num_beams; ++i) {
    msg.beam_directions[i] = (*mainImage.m_vecBearingTable)[i];
}
```

## Key Points

- GLF and marine_acoustic_msgs use identical beam-major data layout
- Beam angles from `m_vecBearingTable` are factory-calibrated
- No data reorganization needed for conversion
- Direct memory copy is the most efficient approach

