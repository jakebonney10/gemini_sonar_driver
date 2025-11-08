# Gemini Sonar Driver

ROS2 driver for the Tritech Gemini 1200ikd multibeam imaging sonar.

## Features

- ✅ Configure sonar parameters via ROS2 parameters
- ✅ Start/stop sonar operation via ROS2 services
- ✅ Publish multibeam data using `marine_acoustic_msgs`
- ✅ Log data in native Gemini format alongside ROS2 bags
- ✅ Publish raw Gemini SDK packets for debugging

## Dependencies

- ROS2 Humble
- `marine_acoustic_msgs` package
- Gemini SDK v2.0.41.0 (included in this repository)

## Installation

```bash
cd ~/ros/rhody_ws
colcon build --packages-select gemini_sonar_driver_interfaces gemini_sonar_driver
source install/setup.bash
```

## Configuration

Edit `config/gemini_sonar.yaml` to configure sonar parameters:

```yaml
sonar_id: 1              # Sonar ID on network
range_m: 75.0            # Max range in meters
gain_percent: 50.0       # Receiver gain 0-100%
frequency_khz: 720.0     # Operating frequency
num_beams: 512           # Number of beams
```

## Usage

### Launch the Driver

```bash
ros2 launch gemini_sonar_driver gemini_sonar.launch.py
```

### Start the Sonar

```bash
# Start with native logging enabled
ros2 service call /gemini/start_sonar gemini_sonar_driver_interfaces/srv/StartSonar "{enable_logging: true, log_directory: '~/gemini_logs'}"

# Start without native logging
ros2 service call /gemini/start_sonar gemini_sonar_driver_interfaces/srv/StartSonar "{enable_logging: false, log_directory: ''}"
```

### Stop the Sonar

```bash
ros2 service call /gemini/stop_sonar gemini_sonar_driver_interfaces/srv/StopSonar "{save_configuration: false}"
```

### Monitor Topics

```bash
# View sonar images
ros2 topic echo /gemini/sonar_image

# View projection info
ros2 topic echo /gemini/projection

# View raw packets
ros2 topic echo /gemini/raw
```

### Record ROS2 Bags

```bash
ros2 bag record -a  # Record all topics
# or specifically:
ros2 bag record /gemini/sonar_image /gemini/projection
```

## Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/gemini/raw_sonar_image` | `marine_acoustic_msgs/RawSonarImage` | Raw sonar data with beam angles and samples |
| `/gemini/projected_sonar` | `marine_acoustic_msgs/ProjectedSonarImage` | Projected sonar image with cartesian coordinates |
| `/gemini/detections` | `marine_acoustic_msgs/SonarDetections` | Multibeam detections (ranges, angles, intensities) - **recommended for FLS analysis** |
| `/gemini/raw` | `gemini_sonar_driver_interfaces/RawPacket` | Raw Gemini SDK packets for debugging |

**Note:** For multibeam forward-looking sonar (FLS) applications, the `SonarDetections` message is recommended as it provides properly formatted detection data with travel times, beam angles, and intensities suitable for obstacle detection and navigation.

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/gemini/start_sonar` | `StartSonar` | Start sonar pinging |
| `/gemini/stop_sonar` | `StopSonar` | Stop sonar pinging |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sonar_id` | int | 1 | Sonar ID on network |
| `software_mode` | string | "Evo" | SDK mode (Evo/EvoC/SeaNet/SeaNetC) |
| `range_m` | double | 75.0 | Maximum range in meters |
| `gain_percent` | double | 50.0 | Receiver gain 0-100% |
| `sound_speed_ms` | int | 1500 | Sound speed in m/s |
| `frequency_khz` | double | 720.0 | Operating frequency in kHz |
| `num_beams` | int | 512 | Number of beams |
| `bins_per_beam` | int | 1500 | Range cells per beam |
| `beam_spacing_deg` | double | 0.25 | Beam spacing in degrees |

## Troubleshooting

### "Failed to initialize Gemini network"
- Check that no other program is using the Gemini SDK
- Verify sonar is powered and connected to network
- Check `sonar_id` matches your hardware

### No data on topics
- Ensure sonar is started via service call
- Check sonar cable connections
- Verify network configuration

## TODO / Future Features

- [ ] Complete Gemini SDK API integration (start/stop/configure functions)
- [ ] Parse complete ping head/line/tail structures
- [ ] Add parameter validation and bounds checking
- [ ] Implement sonar firmware version detection
- [ ] Add diagnostic publishing for sonar health
- [ ] Create native-to-ROS2 bag conversion utility
- [ ] Add support for dual-frequency operation
- [ ] Implement automatic range/gain adjustment

## License

Apache 2.0

## Maintainer

Jake Bonney (jake@bonrov.com)
