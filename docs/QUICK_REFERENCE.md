# Gemini 1200ik Driver - Quick Reference

## 🚀 Quick Start

```bash
# 1. Build
cd ~/ros/rhody_ws
colcon build --packages-select gemini_sonar_driver --symlink-install
source install/setup.bash

# 2. Configure (edit config/gemini_sonar.yaml)
range_m: 75.0
gain_percent: 50.0
sound_speed_ms: 1500
num_beams: 512

# 3. Launch
ros2 launch rhody navigation.launch.py

# 4. Start sonar
ros2 service call /gemini/start_sonar gemini_sonar_driver_interfaces/srv/StartSonar

# 5. Monitor
ros2 topic hz /gemini/raw_sonar_image
ros2 topic echo /gemini/detections --once

# 6. Stop sonar
ros2 service call /gemini/stop_sonar gemini_sonar_driver_interfaces/srv/StopSonar
```

## 📡 Topics

| Topic | Message Type | Rate | Description |
|-------|-------------|------|-------------|
| `/gemini/raw_sonar_image` | `marine_acoustic_msgs/RawSonarImage` | 10 Hz | Full beam data with timing |
| `/gemini/projected_sonar` | `marine_acoustic_msgs/ProjectedSonarImage` | 1 Hz | Cartesian projection |
| `/gemini/detections` | `marine_acoustic_msgs/SonarDetections` | 10 Hz | Multibeam detections (recommended) |
| `/gemini/raw` | `gemini_sonar_driver_interfaces/RawPacket` | 10 Hz | Raw SDK packets |

## ⚙️ Parameters

```yaml
# Essential
sonar_id: 1                    # Sonar ID (usually 1)
range_m: 75.0                  # Range: 1-120m
gain_percent: 50.0             # Gain: 0-100%
sound_speed_ms: 1500           # SOS: 1400-1600 m/s

# Image Quality
num_beams: 512                 # Beams: 256, 512, or 1024
bins_per_beam: 1500            # Range cells per beam
beam_spacing_deg: 0.25         # Angular resolution

# Topics (defaults shown)
topics:
  raw_sonar_image: "gemini/raw_sonar_image"
  projected_sonar_image: "gemini/projected_sonar"
  sonar_detections: "gemini/detections"
  raw_packet: "gemini/raw"
```

## 🔧 Services

```bash
# Start sonar
ros2 service call /gemini/start_sonar \
  gemini_sonar_driver_interfaces/srv/StartSonar

# Stop sonar
ros2 service call /gemini/stop_sonar \
  gemini_sonar_driver_interfaces/srv/StopSonar
```

## 🐛 Common Issues

### No Sonar Data
```bash
# Check sonar discovery
ros2 topic echo /rosout | grep -i "gemini\|svs5"

# Verify network
ping 192.168.0.1  # Or your sonar's IP
sudo netstat -ulnp | grep 50002  # Check UDP port
```

### Library Not Found
```bash
# Add to ~/.bashrc
export LD_LIBRARY_PATH=/path/to/GeminiSDK/bin:$LD_LIBRARY_PATH
```

### Another Instance Running
```bash
# Kill other Tritech software
pkill -f "Seanet\|GemView\|Svs5"
```

## 📊 Monitoring Commands

```bash
# Data rates
ros2 topic hz /gemini/raw_sonar_image
ros2 topic hz /gemini/detections

# Message inspection
ros2 topic echo /gemini/raw_sonar_image/ping_info
ros2 topic echo /gemini/detections/intensities | head -20

# Live bandwidth
ros2 topic bw /gemini/raw_sonar_image

# Node info
ros2 node info /gemini_sonar_driver
```

## 🎯 Typical Settings by Environment

### **Clear Water (harbor, lake)**
```yaml
range_m: 75.0
gain_percent: 40.0
sound_speed_ms: 1485  # Fresh: 1450, Salt: 1500
num_beams: 512
```

### **Turbid Water (river, coastal)**
```yaml
range_m: 50.0
gain_percent: 60.0
sound_speed_ms: 1500
num_beams: 512
```

### **Close Range (docking, inspection)**
```yaml
range_m: 20.0
gain_percent: 30.0
sound_speed_ms: 1500
num_beams: 1024  # Max resolution
```

### **Long Range (survey)**
```yaml
range_m: 120.0
gain_percent: 70.0
sound_speed_ms: 1500
num_beams: 256  # Lower res for speed
```

## 🔍 Debugging

```bash
# Increase logging level
ros2 run gemini_sonar_driver gemini_sonar_node_main --ros-args \
  --log-level debug

# Check errors
ros2 topic echo /rosout --field msg | grep -i "error\|fail\|warn"

# Raw packets
ros2 topic echo /gemini/raw | head -5
```

## 📚 Documentation Files

- `SVS5_SEQUENCER_MIGRATION.md` - API migration guide
- `GLF_DATA_FORMAT.md` - Data structure reference
- `COMPLETE_INTEGRATION_SUMMARY.md` - Full integration docs
- `SDK_INTEGRATION.md` - Network architecture
- `GEMINI_1200IK_CONFIG.md` - Hardware-specific config

## 🌐 Network Setup

```bash
# Configure ethernet interface for 192.168.0.x
sudo ip addr add 192.168.0.100/24 dev eth0
sudo ip link set eth0 up

# Or use NetworkManager
nmcli con add type ethernet ifname eth0 ip4 192.168.0.100/24

# Verify
ip addr show eth0
```

## 📦 Message Structure Quick Reference

### RawSonarImage
```
header (timestamp, frame_id)
ping_info (frequency, sound_speed)
sample_rate
samples_per_beam
sample0
tx_delays[]
tx_angles[]
rx_angles[]  # Beam angles
image (beam_count, data[])
```

### SonarDetections
```
header
ping_info
flags[]  # Detection quality
two_way_travel_times[]
intensities[]
tx_delays[]
tx_angles[]
rx_angles[]
```

## 🎨 Visualization (RViz2)

```bash
# Install marine plugins (if not already)
sudo apt install ros-humble-marine-visualization

# Launch RViz2
ros2 run rviz2 rviz2

# Add displays:
# - MarineAcoustic/RawSonarImage
# - MarineAcoustic/ProjectedSonarImage
# - MarineAcoustic/SonarDetections
```

## 💡 Pro Tips

1. **Start with low gain** (30-40%) and increase if needed
2. **Use 512 beams** for balanced quality/performance
3. **Check sound speed** for your water temperature/salinity
4. **Monitor frame rate** - should be 8-12 Hz
5. **Use SonarDetections** for most navigation tasks
6. **Log raw data** during important missions
7. **Test in pool** before ocean deployment
8. **Keep firmware updated** (check Tritech website)

## 🔗 Useful Links

- Tritech Support: https://www.tritech.co.uk/support
- Marine Acoustic Msgs: https://github.com/apl-ocean-engineering/marine_acoustic_msgs
- ROS2 Humble Docs: https://docs.ros.org/en/humble/

---

**Need help?** Check the full documentation in `docs/` directory.
