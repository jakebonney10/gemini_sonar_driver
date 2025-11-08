# Gemini SDK Integration Reference

This document describes how the Gemini sonar driver integrates with the Tritech Gemini SDK v2.0.41.0.

## Network Architecture & Connection

### Network Topology

The Gemini SDK communicates with the sonar over **Ethernet using UDP**:

```
PC (192.168.0.x)  ←→  Network Switch  ←→  Gemini Sonar (192.168.0.y)
    ↑                                              ↑
    └── Port 50002 (SDK listens/sends)           └── Fixed/Alt IP
```

### IP Address Modes

Gemini sonars support two IP address configurations:

#### 1. Fixed IP Address (Factory Default)
- **Primary:** `192.168.0.200` + Sonar ID offset
- **Example:** Sonar ID 1 → `192.168.0.201`, Sonar ID 2 → `192.168.0.202`
- **Subnet:** `255.255.255.0`
- **Cannot be changed** (factory configured)

#### 2. Alternate IP Address (User Configurable)
- Can be set to any valid IP address
- Configured via SDK: `GEMX_SetAltSonarIPAddress()`
- Stored in sonar's non-volatile memory
- Useful for integrating into existing networks

### Network Discovery Process

When `GEM_StartGeminiNetworkWithResult()` is called:

1. **Opens UDP socket** on port `50002` (hardcoded in SDK)
2. **Binds to all interfaces** (`INADDR_ANY`) to listen for broadcasts
3. **Starts receive thread** to process incoming UDP packets
4. **Waits for status broadcasts** from sonar
5. **Identifies sonar** by Sonar ID in status packet
6. **Establishes bidirectional communication**

```cpp
// Initialization sequence
GEM_SetGeminiSoftwareMode("Evo");           // Set before starting network
GEM_SetHandlerFunction(&callback);          // Register callback
int result = GEM_StartGeminiNetworkWithResult(1, false);  // sonarID=1
// result: 1=success, 0=failure (port in use)
```

### Broadcast vs Unicast

The sonar can operate in two transmission modes:

#### Status Broadcasts (Default: ENABLED)
- Sonar sends **status packets** to broadcast address every 1 second
- Destination: `192.168.0.255` (or subnet broadcast)
- Allows multiple PCs to see the sonar
- Discovery mechanism for SDK

#### Ping Data (Default: UNICAST)
- Ping data (head/line/tail) sent to **specific PC that requested ping**
- Prevents network flooding with high-bandwidth data
- Can be changed to broadcast with `GEMX_SetAltSonarIPAddressWithFlag()`

### Port Configuration

| Component | Port | Protocol | Direction | Purpose |
|-----------|------|----------|-----------|---------|
| **SDK** | 50002 | UDP | Listen | Receive all sonar messages |
| **SDK** | 50002 | UDP | Send | Send commands to sonar |
| **Sonar** | 50002 | UDP | Send | Status broadcasts |
| **Sonar** | 50002 | UDP | Send | Ping data (to requestor) |

**Critical:** Only **one process** can bind to port 50002 at a time. Running multiple SDK instances will fail with "port already in use."

### Network Configuration Functions

#### Get/Set Alternate IP
```cpp
// Get current alternate IP from sonar
unsigned char a1, a2, a3, a4;  // IP octets
unsigned char s1, s2, s3, s4;  // Subnet mask octets
GEMX_GetAltSonarIPAddress(sonar_id, &a1, &a2, &a3, &a4, &s1, &s2, &s3, &s4);
// Example result: 192.168.2.201 / 255.255.255.0

// Set new alternate IP on sonar (stored in NV memory)
GEMX_SetAltSonarIPAddress(sonar_id, 192, 168, 2, 201, 255, 255, 255, 0);
```

#### Use Alternate IP
```cpp
// Tell SDK to communicate using alternate IP instead of fixed IP
GEMX_UseAltSonarIPAddress(sonar_id, 192, 168, 2, 201, 255, 255, 255, 0);

// Or toggle between fixed and alternate
GEMX_TxToAltIPAddress(sonar_id, 1);  // 1=use alt, 0=use fixed
```

#### Configure Broadcast Options
```cpp
// Advanced: Control broadcast behavior
GEMX_SetAltSonarIPAddressWithFlag(
    sonar_id,
    3232236233,    // IP as uint32 (192.168.2.201)
    4294967040,    // Subnet as uint32 (255.255.255.0)
    0x0,           // broadcastStatus: 0x0=enabled, 0x01234567=disabled
    0x01234567     // broadcastPing: 0x0=disabled, 0x01234567=enabled (720is only)
);
```

### Multi-Sonar Support

The SDK can communicate with multiple sonars simultaneously:

```cpp
// Sonar discovery
unsigned short sonar_list[10];
int num_sonars = GEMX_GetSonars(sonar_list);
// Returns: Number of sonars found, populates sonar_list with IDs

// Each sonar identified by unique Sonar ID
// SDK internally routes messages based on ID
GEM_StartGeminiNetworkWithResult(1, false);  // Talk to sonar ID 1
GEM_StartGeminiNetworkWithResult(2, false);  // ERROR: Port already bound!
```

**Note:** `GEM_StartGeminiNetworkWithResult()` is called **once** and discovers **all sonars** on the network. The sonar ID parameter is primarily for internal tracking.

### Connection Troubleshooting

#### "Failed to initialize network" (Return 0)
**Causes:**
- Another SDK instance running (e.g., Tritech GUI app)
- Port 50002 already bound by another process
- Firewall blocking UDP port 50002

**Solutions:**
```bash
# Check what's using port 50002
sudo netstat -tulpn | grep 50002
# or
sudo lsof -i :50002

# Kill competing process
kill -9 <PID>

# Check firewall
sudo ufw status
sudo ufw allow 50002/udp
```

#### No Status Messages Received
**Causes:**
- Sonar powered off
- Network cable disconnected
- Wrong subnet (PC and sonar not on same network)
- Sonar in bootloader mode (firmware update)

**Verify:**
```bash
# Check network interface is up
ip addr show

# Ping sonar (may not respond, but checks connectivity)
ping 192.168.0.201

# Use tcpdump to see UDP traffic
sudo tcpdump -i eth0 port 50002
```

#### Network Interface Selection
If PC has multiple network interfaces, SDK binds to **all interfaces** (`INADDR_ANY`). To force specific interface:

```bash
# Use routing table to prioritize sonar subnet
sudo ip route add 192.168.0.0/24 dev eth0
```

Or configure PC network interface to match sonar subnet:
```bash
sudo ip addr add 192.168.0.100/24 dev eth0
sudo ip link set eth0 up
```

### Bandwidth Considerations

Gemini sonar generates significant network traffic:

| Configuration | Data Rate | Notes |
|---------------|-----------|-------|
| 256 beams, 10 Hz | ~15 MB/s | Standard operation |
| 512 beams, 10 Hz | ~30 MB/s | High resolution |
| 512 beams, 20 Hz | ~60 MB/s | Maximum performance |

**Network Requirements:**
- **Minimum:** 100 Mbps Ethernet (100BASE-T)
- **Recommended:** 1 Gbps Ethernet (1000BASE-T)
- Use **Cat5e or better** cables
- Avoid WiFi (latency and bandwidth issues)

### Network Performance Tuning

For high-performance applications:

```bash
# Increase UDP receive buffer size
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400

# Check for packet drops
netstat -su | grep "packet receive errors"
```

In code, consider processing data in separate thread to avoid blocking SDK receiver:
```cpp
void processGeminiData(int type, int length, char* data) {
    // Quick copy to queue, process in worker thread
    std::lock_guard<std::mutex> lock(queue_mutex_);
    data_queue_.push(DataPacket{type, length, std::vector<char>(data, data+length)});
    cv_.notify_one();
}
```

## SDK Architecture

The Gemini SDK provides two levels of API:

1. **Low-level GEMX API** (`GeminiCommsPublic.h`) - Direct hardware control
2. **High-level Sequencer API** (`Svs5SequencerApi.h`) - Simplified image processing interface

**Our driver uses the low-level GEMX API** for maximum control and ROS2 integration flexibility.

## Initialization Sequence

```cpp
// 1. Set software mode (Evo, EvoC, SeaNet, SeaNetC)
GEM_SetGeminiSoftwareMode("Evo");

// 2. Set callback for receiving data
GEM_SetHandlerFunction(&GeminiSonarNode::geminiDataCallback);

// 3. Start network interface
int result = GEM_StartGeminiNetworkWithResult(sonar_id, false);

// 4. Configure sonar parameters
GEMX_AutoPingConfig(sonar_id, range_m, gain_percent, sound_speed_ms);
GEMX_SetGeminiBeams(sonar_id, num_beams);
GEMX_SendGeminiPingConfig(sonar_id);
```

## Software Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| **Evo** | Raw unprocessed beam data (CGemPingHead, CGemPingLine, CGemPingTailExtended) | Maximum flexibility, custom processing |
| **EvoC** | Raw data with range compression | Reduced bandwidth, maintains raw format |
| **SeaNet** | Processed bearing data (CGemBearingData) | Legacy compatibility |
| **SeaNetC** | Processed with compression | Legacy with reduced bandwidth |

**Our driver uses Evo mode** to receive raw beam data for conversion to marine_acoustic_msgs.

## Data Flow

```
Gemini Sonar Hardware
        ↓
Gemini SDK (via Ethernet)
        ↓
geminiDataCallback() [static]
        ↓
processGeminiData() [instance method]
        ↓
    ┌───┴────┬─────────┬──────────┐
    ↓        ↓         ↓          ↓
PingHead  PingLine  PingLine  PingTail
(metadata) (beam 1)  (beam 2)  (complete)
    ↓        ↓         ↓          ↓
processPingHead()  processPingLine() × N  processPingTail()
    ↓                   ↓                       ↓
Store metadata    Accumulate beams    Trigger conversion
                                              ↓
                                conversions::createRawSonarImage()
                                conversions::createProjectedSonarImage()
                                conversions::createSonarDetections()
                                              ↓
                                        Publish to ROS2
```

## Message Types

The SDK sends data via callback with these message types:

| Type | Value | Structure | Description |
|------|-------|-----------|-------------|
| `PING_HEAD` | 0 | `CGemPingHead` | Ping metadata (range, SOS, timestamp) |
| `PING_DATA` | 1 | `CGemPingLine` | Individual beam intensity data |
| `PING_TAIL` | 2 | `CGemPingTail` | Ping completion marker |
| `GEM_STATUS` | 3 | `CGemStatusPacket` | Sonar health/status |

## Key SDK Functions Used

### Network Control
```cpp
GEM_StartGeminiNetworkWithResult(sonar_id, hardReset)
// Returns: 1 = success, 0 = failure (port already in use)

GEM_StopGeminiNetwork()
// Clean shutdown of network interface
```

### Sonar Configuration
```cpp
GEMX_AutoPingConfig(sonar_id, range_m, gain_percent, sound_speed_ms)
// Sets: Range (0.1-150m), Gain (0-100%), Speed of Sound (1400-1600 m/s)

GEMX_SetGeminiBeams(sonar_id, num_beams)
// Sets: Number of beams (256 or 512 for most systems)

GEMX_SendGeminiPingConfig(sonar_id)
// Sends configuration to sonar (required after parameter changes)
```

### Ping Control
```cpp
GEMX_SetPingMode(sonar_id, mode)
// mode = 0: Single ping on config receipt
// mode = 1: Continuous pinging at fixed interval

GEMX_SetInterPingPeriod(sonar_id, period_microseconds)
// Sets time between pings (0-999000 microseconds)
// Default: 100000 µs (100ms = 10 Hz ping rate)
```

## Data Structures

### CGemPingHead
```cpp
class CGemPingHead {
    unsigned short m_pingID;              // Ping sequence number
    unsigned short m_extMode;             // Extended mode flags
    unsigned int   m_transmitTimestampL;  // Timestamp low 32 bits
    unsigned int   m_transmitTimestampH;  // Timestamp high 32 bits
    unsigned short m_startRange;          // Start range (bins)
    unsigned short m_endRange;            // End range (bins)
    unsigned int   m_lineTime;            // Time per beam line
    unsigned short m_numBeams;            // Number of beams in ping
    unsigned short m_numChans;            // Number of channels
    unsigned char  m_sampChan;            // Samples per channel
    unsigned char  m_baseGain;            // Base gain setting
    unsigned short m_spdSndVel;           // Speed of sound used
    unsigned short m_sosUsed;             // Actual SOS value (m/s)
    unsigned char  m_RLEThresholdUsed;    // Run-length encoding threshold
    unsigned char  m_rangeCompressionUsed;// Range compression factor
};
```

### CGemPingLine
```cpp
class CGemPingLine {
    unsigned char  m_gain;         // Gain for this beam
    unsigned char  m_pingID;       // Ping ID (matches head)
    unsigned short m_lineID;       // Beam number
    unsigned short m_scale;        // Scale factor
    unsigned short m_lineInfo;     // Width encoded in bits 7-15
    unsigned char  m_startOfData;  // First byte of beam data
    
    int GetLineWidth() const {     // Helper to extract width
        return ((m_lineInfo & 0xff80) >> 7) * 4;
    }
};
```

### CGemPingTail
```cpp
class CGemPingTail {
    unsigned char  m_pingID;  // Ping ID (matches head)
    unsigned char  m_flags;   // Status flags
    unsigned short m_spare;   // Reserved
};
```

## Timestamp Handling

Gemini timestamps are 64-bit microsecond values split across two 32-bit fields:

```cpp
uint64_t timestamp_us = (static_cast<uint64_t>(m_transmitTimestampH) << 32) | 
                        m_transmitTimestampL;
double timestamp_seconds = timestamp_us / 1000000.0;

// Convert to ROS2 time
rclcpp::Time ros_time = this->now();  // Use ROS clock instead of sonar time
```

**Note:** We use ROS2 clock (`this->now()`) for message timestamps rather than sonar timestamps to ensure proper synchronization with other ROS2 nodes.

## Range Calculation

```cpp
// From ping head structure
double range_meters = (m_endRange - m_startRange) * (m_sosUsed / 2000.0);

// The division by 2000 accounts for:
// - Round-trip time (divide by 2)
// - Conversion from bins to meters (depends on sampling rate)
```

## Beam Data Extraction

```cpp
const CGemPingLine* ping_line = reinterpret_cast<const CGemPingLine*>(data);
int line_width = ping_line->GetLineWidth();  // Number of samples

// Data starts immediately after the header structure
const uint8_t* beam_data = reinterpret_cast<const uint8_t*>(&ping_line->m_startOfData);

// Extract to vector
std::vector<uint8_t> beam_samples(beam_data, beam_data + line_width);
```

## Common Issues & Solutions

### "Failed to initialize Gemini network"
**Cause:** Another program is using port 50002 (Gemini comms port)
**Solution:** 
```bash
# Check for processes using the port
sudo netstat -tulpn | grep 50002

# Kill competing process
kill -9 <PID>
```

### "Sonar not pinging"
**Check:**
1. SDK initialized successfully?
2. Ping mode set to continuous (mode 1)?
3. `GEMX_SendGeminiPingConfig()` called?
4. Sonar powered and connected?

### "No beam data received"
**Verify:**
1. Callback function registered correctly
2. Software mode set to "Evo" (raw data)
3. Static instance pointer set before SDK init
4. Callback is thread-safe (uses mutex for shared data)

## Performance Tuning

### Ping Rate
```cpp
// Fast pinging (20 Hz)
GEMX_SetInterPingPeriod(sonar_id, 50000);  // 50ms

// Standard rate (10 Hz)
GEMX_SetInterPingPeriod(sonar_id, 100000); // 100ms

// Slow pinging (5 Hz)
GEMX_SetInterPingPeriod(sonar_id, 200000); // 200ms
```

### Beam Configuration
```cpp
// Lower CPU usage, lower resolution
GEMX_SetGeminiBeams(sonar_id, 256);

// Higher resolution, higher CPU usage
GEMX_SetGeminiBeams(sonar_id, 512);
```

### Range Compression
```cpp
// Reduce bandwidth at long ranges
GEMX_SetRangeCompression(sonar_id, 
    2,  // compression_level: 0-4 (0=none, 4=16x)
    1   // compression_type: 0=average, 1=peak
);
```

## Thread Safety

The SDK callback runs in a separate thread created by the SDK. Our implementation uses:

1. **Static callback wrapper** - `geminiDataCallback()` is static
2. **Instance pointer** - Static `instance_` pointer to access node methods
3. **Mutex protection** - `data_mutex_` protects `current_ping_beams_`
4. **Thread-safe publishers** - ROS2 publishers are inherently thread-safe

```cpp
// SDK thread
static void geminiDataCallback(int type, int length, char* data) {
    if (instance_ != nullptr) {
        instance_->processGeminiData(type, length, data);  // Instance method
    }
}

// Instance method with mutex protection
void processGeminiData(int type, int length, char* data) {
    std::lock_guard<std::mutex> lock(data_mutex_);  // Thread-safe
    // ... process data ...
}
```

## Reference Links

- **SDK Version:** Gemini SDK v2.0.41.0 Ubuntu 22.04 x86_64
- **Header Files:** 
  - `GeminiCommsPublic.h` - Low-level API
  - `GeminiStructuresPublic.h` - Data structures
  - `Svs5SequencerApi.h` - High-level API (not used)
- **Example Code:** `GeminiSDK_v2.0.41.0_Ubuntu_22.04_x86_64/src/GeminiSDKConsoleApp/`

## Implementation Status

✅ **Completed:**
- Network initialization and shutdown
- Sonar configuration (range, gain, SOS, beams)
- Ping control (start/stop continuous pinging)
- Data callback handling
- Ping head parsing (metadata extraction)
- Ping line parsing (beam data extraction)
- Ping tail handling (trigger message conversion)
- Integration with conversions module
- ROS2 message publishing

🔄 **Future Enhancements:**
- Status message handling (sonar health monitoring)
- Dynamic reconfiguration (update parameters while running)
- Advanced features (chirp mode, range compression, high resolution)
- Multi-sonar support (handle multiple sonar IDs)
