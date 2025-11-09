# Gemini MK2 Beamformer Board Status Message
# Contains health and diagnostic information from the beamformer FPGA
# Used by 720ik, 1200ik, and other MK2 platform sonars

std_msgs/Header header

# Identification
uint16 sonar_id                    # Sonar device ID
uint16 firmware_version            # Firmware version
uint16 transducer_frequency        # Transducer frequency in kHz

# Network Configuration
string ip_address                  # Sonar IP address
string subnet_mask                 # Subnet mask
string mac_address                 # MAC address (XX:XX:XX:XX:XX:XX format)
uint16 link_type                   # Link type (0=1000BaseT, 1=VDSL, etc.)
uint16 vdsl_downstream_speed       # VDSL downstream speed (Mbps)
uint16 vdsl_upstream_speed         # VDSL upstream speed (Mbps)

# Power Supply Voltages (Volts)
float32 vcc_internal_v             # Internal voltage
float32 vcc_auxiliary_v            # Auxiliary voltage

# Temperature Readings (Celsius)
float32 die_temp                   # FPGA die temperature
float32 pcb_temp                   # PCB temperature
float32 comms_temp                 # Communications section temperature
float32 tx_temp                    # Transmit section temperature
float32 psu_temp                   # Power supply temperature

# Shutdown Temperature Thresholds (Celsius)
float32 die_shutdown_temp          # FPGA die shutdown threshold
float32 pcb_shutdown_temp          # PCB shutdown threshold
float32 comms_shutdown_temp        # Communications shutdown threshold
float32 tx_shutdown_temp           # Transmit shutdown threshold
float32 psu_shutdown_temp          # PSU shutdown threshold

# Status Information
uint16 status_flags                # General status flags
uint16 shutdown_status             # Shutdown status bits
bool die_over_temp                 # Die temperature exceeded threshold
uint32 bootsts_register            # Boot status register
uint16 dip_switch                  # DIP switch settings

# Timestamp
uint64 sonar_timestamp             # Sonar internal timestamp (microseconds)

# Health Summary (computed)
bool healthy                       # Overall health status (all temps OK, no errors)
string health_message              # Human-readable health status message
