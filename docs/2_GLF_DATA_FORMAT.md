# GLF (Gemini Log Format) Data Reference

## Overview

The Gemini SDK uses **GLF (Gemini Log Format)** to deliver processed, beamformed sonar images via the Svs5Sequencer API.

## Key GLF Structure

```
GLF::GLogTargetImage
├── GLF::GMainImage m_mainImage
│   ├── std::vector<UInt8>* m_vecData (intensity values, beam-major order)
│   ├── std::vector<double>* m_vecBearingTable (beam angles in radians)
│   ├── UInt32 m_uiStartRange / m_uiEndRange
│   ├── UInt32 m_uiModulationFrequency (Hz)
│   ├── float m_fSosAtXd (sound speed)
│   ├── Int16 m_sPercentGain
│   ├── double m_dbTxTime (timestamp)
│   └── UInt8 m_ucPlatform (sonar model ID)
└── GLF::GZoomImage m_aczImage (optional acoustic zoom)
```

## Data Layout

Image data in `m_vecData` is organized in **beam-major order**:

```
[beam0_samples, beam1_samples, ..., beamN_samples]
```

**Calculate dimensions:**
```cpp
num_beams = m_vecBearingTable->size()
samples_per_beam = m_vecData->size() / num_beams
```

**Access specific sample:**
```cpp
index = beam_idx * samples_per_beam + sample_idx
intensity = (*m_vecData)[index]
```

## Essential Metadata Fields

| Field | Type | Description |
|-------|------|-------------|
| `m_vecBearingTable` | `vector<double>` | Beam angles in radians |
| `m_uiModulationFrequency` | `UInt32` | Operating frequency (Hz) |
| `m_fSosAtXd` | `float` | Sound speed (m/s) |
| `m_sPercentGain` | `Int16` | Receiver gain (0-100%) |
| `m_dbTxTime` | `double` | Transmit timestamp |
| `m_ucPlatform` | `UInt8` | Sonar model (4 = Gemini 1200ik) |

## Usage Example

```cpp
// Receive and parse GLF message
GLF::GLogTargetImage* image = (GLF::GLogTargetImage*)value;
const GLF::GMainImage& mainImage = image->m_mainImage;

// Extract dimensions
size_t num_beams = mainImage.m_vecBearingTable->size();
size_t samples_per_beam = mainImage.m_vecData->size() / num_beams;

// Access beam data
for (size_t beam = 0; beam < num_beams; ++beam) {
    double angle = (*mainImage.m_vecBearingTable)[beam];
    for (size_t sample = 0; sample < samples_per_beam; ++sample) {
        size_t idx = beam * samples_per_beam + sample;
        uint8_t intensity = (*mainImage.m_vecData)[idx];
    }
}
```

## References

- GLF Structure: `GenesisSerializer/GlfLoggerGeminiStructure.h`
- SDK API: `Svs5Seq/Svs5SequencerApi.h`
