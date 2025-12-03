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

Put gemini sdk in gemini_sonar_driver top level directory. Make sure to run `InstallSDK.sh` to install the libs system wide.

```bash
cd ~/ros/rhody_ws
colcon build --packages-select gemini_sonar_driver_interfaces gemini_sonar_driver
source install/setup.bash
```

[NOTE!] For some reason on 24.04 and Jazzy the node can fail to find the gemini SDK libs at runtime. If this happpens you can manually source these in the terminal prior to running the node it termporaily fixes the problem. You could also add to .bashrc so it persists. 

```bash
export GEMINI_BIN="/path/to/ros_workspace/src/gemini_sonar_driver/GeminiSDK_v2.0.39.0_Ubuntu_24.04_x86_64/bin"
export LD_LIBRARY_PATH="$GEMINI_BIN:$LD_LIBRARY_PATH"
```

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

You can use acoustic_msgs_tools acoustic_image_view to visualisze raw_sonar_image msgs. 

```ros2 run acoustic_msgs_tools acoustic_image_view```

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

## Published Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/gemini/raw_sonar_image` | `marine_acoustic_msgs/RawSonarImage` | Raw sonar data with beam angles and samples |
| `/gemini/raw` | `gemini_sonar_driver_interfaces/RawPacket` | Raw Gemini SDK packets for debugging |
| `/gemini/status` | `gemini_sonar_driver_interfaces/GeminiStatus` | Sonar status information |
| `/gemini/logger_status` | `gemini_sonar_driver_interfaces/LoggerStatus` | Native GLF logger status |

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
| `range_m` | double | 5.0 | Maximum range in meters |
| `gain_percent` | double | 100.0 | Receiver gain 0-100% |
| `sound_speed_ms` | int | 1500 | Sound speed in m/s |

## Troubleshooting

To run with verbosity output set to DEBUG use 
```ros2 run gemini_sonar_driver gemini_sonar_node --ros-args --log-level gemini_sonar_driver:=debug```

### "Failed to initialize Gemini network"
- Check that no other program is using the Gemini SDK
- Verify sonar is powered and connected to network
- Check `sonar_id` matches your hardware or use id=0 if unknown

## TODO / Future Features

- [ ] make raw_msg type publisher optional 
- [ ] Create glf-to-ROS2 conversions to play back log files
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
