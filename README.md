# Gemini Sonar Driver

ROS2 driver for the Tritech Gemini 1200ikd multibeam imaging sonar.

## Features

- Configure sonar parameters via ROS2 parameters
- Start/stop sonar operation via ROS2 services
- Publish multibeam data using `marine_acoustic_msgs` RawSonarImage msg
- Log data in native Gemini .glf format
- Publish raw Gemini SDK packets for debugging

## Dependencies

- ROS2 Humble or Jazzy
- `marine_acoustic_msgs` package
- Gemini SDK v2.0.41.0 (v2.0.39.0 should also work)

## Installation

Extract the Gemini SDK into the top level of this repository (next to this README), e.g.:

```
gemini_sonar_driver/
  GeminiSDK_v2.0.41.0_Ubuntu_22.04_x86_64/   <-- extracted SDK
  gemini_sonar_driver/                        <-- driver package
  gemini_sonar_driver_interfaces/
```

The SDK is not committed to this repository (Tritech's license does not permit
redistribution), so this is a one-time manual step per machine. The build
auto-detects the newest `GeminiSDK_v*` directory. Do **not** run the SDK's
`InstallSDK.sh` — no system-wide install is needed.

```bash
cd ~/your/ros/workspace
rosdep install --from-paths src -y --ignore-src   # also installs patchelf, needed at build time
colcon build --packages-select gemini_sonar_driver_interfaces gemini_sonar_driver
source install/setup.bash
```

The build copies the SDK libraries into the install space and links the nodes
with relative RPATHs, so the result is fully self-contained: no
`LD_LIBRARY_PATH` setup, no `/usr/local/lib` installs, and `install/` can be
copied to another machine (same OS/arch) and run as-is.

## Configuration

Edit `config/gemini_sonar.yaml` to configure sonar parameters:

```yaml
sonar_id: 0              # Sonar ID on network (0 WORKS FOR ALL SONARS IF ID IS NOT KNOWN)
software_mode: "Evo"     # SDK mode (Evo/EvoC/SeaNet/SeaNetC)
range_m: 5.0             # Max range in meters
gain_percent: 100.0      # Receiver gain 0-100%
sound_speed_ms: 1500     # Sound speed in m/s
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
ros2 service call /gemini/stop_sonar gemini_sonar_driver_interfaces/srv/StopSonar 
```

### Monitor Topics

You can use `acoustic_image_view` from the [acoustic_msgs_tools](https://github.com/k2oceanic/acoustic_msgs_tools) package to visualize `raw_sonar_image` messages.

```bash
ros2 run acoustic_msgs_tools acoustic_image_view
```

You can also echo the raw topics.

```bash
# View sonar images
ros2 topic echo /gemini/raw_sonar_image --once

# View gemini status msgs
ros2 topic echo /gemini/status

# View raw packets
ros2 topic echo /gemini/raw
```

### Record ROS2 Bags

```bash
ros2 bag record -a  # Record all topics
# or specifically:
ros2 bag record /gemini/raw_sonar_image
```

### Convert GLF Files to ROS2 Bags (Offline Processing)

The `glf_to_rosbag` tool converts Gemini native GLF (Gemini Log Format) files recorded during sonar operation into ROS2 bag files for offline replay and analysis.

#### Basic Usage

```bash
ros2 run gemini_sonar_driver glf_to_rosbag <input_glf_file> <output_bag_directory> [frame_id]
```

#### Example

```bash
# Convert a single GLF file to ROS2 bag in MCAP format
ros2 run gemini_sonar_driver glf_to_rosbag \
    /home/user/data/log_2026-01-06-213121.glf \
    /home/user/bags/gemini_replay \
    gemini
```

#### Output

- **Format**: MCAP (modern ROS2 bag format)
- **Topics**: 
  - `/gemini/raw_sonar_image` - Full sonar images with beam data preserved from GLF
  - `/gemini/status` - Sonar health and configuration status messages
- **Timestamps**: Original acquisition timestamps from GLF file are preserved

#### Playing Back Converted Bags

```bash
# Play the converted bag file
ros2 bag play /home/user/bags/gemini_replay/gemini_replay_0.mcap

# View bag information
ros2 bag info /home/user/bags/gemini_replay/gemini_replay_0.mcap

# Visualize during playback (in separate terminal)
ros2 run acoustic_msgs_tools acoustic_image_view
```

## Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/gemini/raw_sonar_image` | `marine_acoustic_msgs/RawSonarImage` | Raw sonar data with beam angles and samples |
| `/gemini/raw` | `gemini_sonar_driver_interfaces/RawPacket` | Raw Gemini SDK packets (optional, for debugging) |
| `/gemini/status` | `gemini_sonar_driver_interfaces/GeminiStatus` | Sonar status information |
| `/gemini/logger_status` | `gemini_sonar_driver_interfaces/LoggerStatus` | Native GLF logger status |

**Note:** The `/gemini/raw` topic is optional and intended for debugging. To disable it in production, set the `topics.raw_packet` parameter to an empty string `""` in your config file.

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/gemini/start_sonar` | `StartSonar` | Start sonar pinging |
| `/gemini/stop_sonar` | `StopSonar` | Stop sonar pinging |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `sonar_id` | int | 0 | Sonar ID on network |
| `software_mode` | string | "Evo" | SDK mode (Evo/EvoC/SeaNet/SeaNetC) |
| `frame_id` | string | "gemini_fls" | TF frame ID for sonar data |
| `log_directory` | string | "/data/gemini" | Directory to save GLF log files |
| `range_m` | double | 5.0 | Maximum range in meters |
| `gain_percent` | double | 100.0 | Receiver gain 0-100% |
| `aperture` | double | 120.0 | Sonar aperture in degrees (120 or 65) |
| `sound_speed_ms` | int | 1500 | Sound speed in m/s |
| `sound_speed_manual` | bool | true | Sound speed mode (true=manual, false=auto) |
| `high_resolution` | bool | true | High resolution mode (1200ik only) |
| `cpu_performance` | int | 2 | SDK beamforming level: 0=LOW, 1=MEDIUM, 2=HIGH, 3=ULTRA |
| `image_quality_pixels` | int | 2048 | Range-direction resolution cap (SDK m_screenPixels). Primary frame-rate lever on embedded CPUs: on RPi CM4 @10m range 2048=~3.7Hz, 1024=~7Hz, 512=~11.5Hz. No effect when range-determined sample count is below the cap (short ranges). |
| `frequency_mode` | int | 0 | Frequency selection (0=auto, 1=low, 2=high, 3=combined) |
| `frequency_auto_threshold_m` | double | 40.0 | Threshold for auto mode LF/HF switching |
| `chirp_mode` | int | 2 | Chirp mode (0=disabled, 1=enabled, 2=auto) |
| `ping_free_run` | bool | false | Continuous pinging (true) vs interval-based (false). NOTE: interval mode measured ~1Hz bursty on RPi CM4; free-run recommended for steady frame rates |
| `ping_interval_ms` | int | 100 | Ping interval in ms when ping_free_run=false |
| `ping_ext_trigger` | bool | false | External TTL hardware trigger (true) vs software (false) |
| `topics.raw_packet` | string | "gemini/raw" | Raw packet topic name (set to "" to disable) |

## Troubleshooting

To run with verbosity output set to DEBUG use 
```ros2 run gemini_sonar_driver gemini_sonar_node --ros-args --log-level gemini_sonar_driver:=debug```

### Low or bursty frame rate (~1 Hz)
- Set `ping_free_run: true` (interval mode was measured to deliver ~1Hz bursts on RPi CM4 while still saturating a CPU core)
- Lower `image_quality_pixels` (2048 -> 1024 or 512). The SDK forms each image on a single thread; per-ping cost scales with samples/beam. Measured on RPi CM4 @10m: 2048=~3.7Hz, 1024=~7Hz, 512=~11.5Hz
- Shorter `range_m` increases achievable rate (less acoustic travel time and fewer samples)
- `cpu_performance` (beam count) is a weak lever; keep at 2 (HIGH)

### "Failed to initialize Gemini network"
- Check that no other program is using the Gemini SDK
- Verify sonar is powered and connected to network and the subnet (i.e 192.168.2.x)
- Check `sonar_id` matches your hardware or use id=0 if unknown

## Docs
For full docs go to [gemini_sonar_driver](https://jakebonney10.github.io/gemini_sonar_driver/).

## TODO / Future Features

- [ ] make raw_msg type publisher optional 
- [x] Create glf-to-ROS2 conversions to play back log files
- [ ] Implement range/gain adjustment on the fly
- [ ] Log ping metadata in custom interfaace msg
- [ ] add marine_acoustic_msgs detections and/or projection msg
- [ ] Add parameter validation and bounds checking
- [ ] Add diagnostic publishing for sonar health
- [ ] Utilize compression/decompression from SDK

## License

Apache 2.0

## Maintainer

Jake Bonney (jake@bonrov.com)
