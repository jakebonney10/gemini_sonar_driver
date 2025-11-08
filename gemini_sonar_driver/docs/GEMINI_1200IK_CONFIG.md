# Gemini 1200ik Configuration Guide

## Your Sonar Model

**Product:** Tritech Gemini 1200ikd  
**Architecture:** MK2  
**Head Type:** `GEM_HEADTYPE_MK2_1200IK` (0x1D)  
**Frequencies:** Dual frequency (1.2 MHz high / low frequency modes)  
**Max Range:** Up to 120m (frequency dependent)  
**Beams:** 256 or 512 configurable

## MK2 Configuration (Current Implementation)

Your driver now uses **MK2-specific SDK functions** optimized for the 1200ik:

### Configuration Sequence

```cpp
// 1. Declare head type (tells SDK what hardware it's communicating with)
GEMX_SetHeadType(sonar_id, GEM_HEADTYPE_MK2_1200IK);

// 2. Auto-calculate optimal transmit pulse for range
GEMX_AutoTXLength(sonar_id, range_m);
// Returns the TX pulse length in cycles
// SDK automatically selects best pulse for your range setting

// 3. Set beam count (256 for lower CPU, 512 for higher resolution)
GEMX_SetGeminiBeams(sonar_id, num_beams);

// 4. Configure gain and speed of sound control
GEMX_SetVelocimeterMode(sonar_id, 
    1,  // gainMode: 1=Manual (use your gain_percent parameter)
    1   // outputMode: 1=Manual SOS (use your sound_speed_ms parameter)
);

// 5. Enable high resolution mode (improves image quality)
GEMX_AutoHighRangeResolution(sonar_id, true);
// 1200kHz: 4mm → 2.4mm resolution at lower ranges

// 6. Configure chirp mode (auto-adapts to range)
GEMX_ConfigureChirpMode(sonar_id, CHIRP_AUTO);
// CHIRP_DISABLED: Always off
// CHIRP_ENABLED: Always on
// CHIRP_AUTO: SDK decides based on range (recommended)

// 7. Send configuration to sonar
GEMX_SendGeminiPingConfig(sonar_id);
```

## Why MK2 Instead of MK1?

| Feature | MK1 (`GEMX_AutoPingConfig`) | MK2 (Individual Functions) |
|---------|------------------------------|---------------------------|
| **Compatibility** | 720i, 720id only | 720is, 720ik, **1200ik** |
| **Configuration** | Single function call | Multiple function calls |
| **Chirp Support** | No | Yes (better range/resolution) |
| **High Resolution** | No | Yes (2.4mm for 1200kHz) |
| **Frequency Control** | Fixed | Auto-switching (1200ik) |
| **Flexibility** | Limited | Full control |

## Gemini 1200ik Specific Features

### Dual Frequency Operation

Your 1200ik automatically switches between high/low frequency based on range:

```cpp
// Configure auto frequency switching (optional)
RangeFrequencyConfig rangeConfig;
rangeConfig.m_frequency = FREQUENCY_AUTO;  // Auto switch
rangeConfig.m_rangeThreshold = 10.0;       // Switch at 10m
GEMX_ConfigureAutoRangeFrequency(sonar_id, rangeConfig);
```

**Frequency Behavior:**
- **< 10m:** High frequency (1.2 MHz) - Better resolution
- **> 10m:** Low frequency - Better range penetration

### Range Capabilities

| Frequency | Max Range | Resolution | Best Use Case |
|-----------|-----------|------------|---------------|
| **1.2 MHz (High)** | ~30m | 2.4mm (high res mode) | Close-range inspection |
| **Low Frequency** | ~120m | 4mm | Long-range surveying |

### Recommended Settings by Application

#### Close-Range Inspection (< 30m)
```yaml
range_m: 20.0
gain_percent: 40.0
num_beams: 512              # Maximum resolution
frequency_khz: 1200.0
sound_speed_ms: 1500        # Adjust for salinity/temp
```

#### Mid-Range Survey (30-60m)
```yaml
range_m: 50.0
gain_percent: 60.0
num_beams: 512
frequency_khz: 1200.0       # Auto-switches to low freq
sound_speed_ms: 1500
```

#### Long-Range Detection (60-120m)
```yaml
range_m: 100.0
gain_percent: 80.0
num_beams: 256              # Lower CPU usage
frequency_khz: 1200.0       # Will use low frequency
sound_speed_ms: 1500
```

## Advanced MK2 Features

### Range Compression

Reduce bandwidth for long-range operations:

```cpp
GEMX_SetRangeCompression(sonar_id,
    2,  // compressionLevel: 0-4 (2 = 4x compression)
    1   // compressionType: 1 = peak (recommended for MK2 chirp)
);
```

**Compression Levels:**
- 0: No compression (full data)
- 1: 2x compression
- 2: 4x compression
- 3: 8x compression
- 4: 16x compression

### Out of Water Override

For testing without submersion:

```cpp
GEMX_SetExtModeOutOfWaterOverride(sonar_id, 1);  // 1 = ping anyway
```

⚠️ **Warning:** Only use for brief tests. Prolonged out-of-water operation may damage transducers.

### Performance Tuning

Adjust ping rate for your CPU capabilities:

```cpp
// In startPinging()
GEMX_SetInterPingPeriod(sonar_id, period_us);
```

**Recommended Rates:**
- **High CPU (512 beams):** 100000 µs (10 Hz) - current default
- **Medium CPU (512 beams):** 200000 µs (5 Hz)
- **Lower CPU (256 beams):** 50000 µs (20 Hz)

## Troubleshooting

### "No data received"

1. **Check head type is correct:**
   ```cpp
   // Verify in initializeGeminiSDK() - should be set automatically
   GEMX_SetHeadType(sonar_id, GEM_HEADTYPE_MK2_1200IK);
   ```

2. **Verify velocimeter mode:**
   - Ensure `GEMX_SetVelocimeterMode()` is called before `GEMX_SendGeminiPingConfig()`

3. **Check frequency settings:**
   - 1200ik auto-switches frequency - don't force incompatible settings

### "Poor image quality"

1. **Enable high resolution:**
   ```cpp
   GEMX_AutoHighRangeResolution(sonar_id, true);
   ```

2. **Adjust gain:**
   - Too low: Weak returns
   - Too high: Noise/saturation
   - Start with 50% and adjust

3. **Use chirp mode:**
   ```cpp
   GEMX_ConfigureChirpMode(sonar_id, CHIRP_AUTO);
   ```

4. **Optimize beam count:**
   - 512 beams for best resolution (higher CPU)
   - 256 beams for faster updates (lower CPU)

### "Slow frame rate"

1. **Increase inter-ping period:**
   ```cpp
   GEMX_SetInterPingPeriod(sonar_id, 200000);  // 5 Hz instead of 10 Hz
   ```

2. **Reduce beam count:**
   ```yaml
   num_beams: 256  # Instead of 512
   ```

3. **Enable range compression:**
   ```cpp
   GEMX_SetRangeCompression(sonar_id, 2, 1);  // 4x compression
   ```

## Network Configuration

Your 1200ik connects via Ethernet:

**Default IP:** `192.168.0.201` (if sonar_id = 1)  
**Port:** 50002 (UDP)  
**Data Rate:** ~30 MB/s (512 beams @ 10 Hz)

**PC Network Setup:**
```bash
# Configure PC to same subnet
sudo ip addr add 192.168.0.100/24 dev eth0
sudo ip link set eth0 up

# Test connectivity (may not respond to ping, but verifies routing)
ping 192.168.0.201

# Watch for sonar broadcasts
sudo tcpdump -i eth0 'udp port 50002'
```

## Parameter Summary

Current configuration in `gemini_sonar.yaml`:

```yaml
/**:
  ros__parameters:
    # Network
    sonar_id: 1                    # Sonar ID (default 1)
    software_mode: "Evo"           # Raw data mode
    
    # Operation (adjust these for your application)
    range_m: 75.0                  # Max 120m for 1200ik
    gain_percent: 50.0             # 0-100%, start with 50
    sound_speed_ms: 1500           # Adjust for water conditions
    frequency_khz: 1200.0          # Auto-switches high/low
    
    # Image Quality
    num_beams: 512                 # 256 or 512
    bins_per_beam: 1500            # Samples per beam
    beam_spacing_deg: 0.25         # Angular resolution
    
    # Topics
    topics:
      raw_sonar_image: "gemini/raw_sonar_image"
      projected_sonar_image: "gemini/projected_sonar"
      sonar_detections: "gemini/detections"
      raw_packet: "gemini/raw"
```

## References

- **Manual:** `0746-SOM-00002-04-Gemini-1200ik-Manual.pdf`
- **SDK Header:** `GeminiCommsPublic.h`
- **Product Page:** https://www.tritech.co.uk/products/gemini-1200ikd
- **SDK Integration:** `docs/SDK_INTEGRATION.md`

## Changelog

**2025-11-07:** Migrated from MK1 (`GEMX_AutoPingConfig`) to MK2 configuration for proper 1200ik support
