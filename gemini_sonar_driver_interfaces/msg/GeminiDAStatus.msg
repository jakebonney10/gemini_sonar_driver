# Gemini MK2 Data Acquisition Board Status Message
# Contains health and diagnostic information from the data acquisition FPGAs
# Used by 720ik, 1200ik, and other MK2 platform sonars
# MK2 sonars can have up to 4 data acquisition boards (DA0, DA1, DA2, DA3)

std_msgs/Header header

# Identification
uint8 board_id                     # Board ID (0=DA0, 1=DA1, 2=DA2, 3=DA3)
uint16 firmware_version            # Firmware version

# Temperature Readings (Celsius)
float32 pcb_temp                   # PCB temperature

# Analog Front End (AFE) Temperatures (Celsius)
# Each DA board has multiple AFE chips for receiving acoustic signals
float32 afe0_top_temp              # AFE 0 top temperature
float32 afe0_bot_temp              # AFE 0 bottom temperature
float32 afe1_top_temp              # AFE 1 top temperature
float32 afe1_bot_temp              # AFE 1 bottom temperature
float32 afe2_top_temp              # AFE 2 top temperature
float32 afe2_bot_temp              # AFE 2 bottom temperature
float32 afe3_top_temp              # AFE 3 top temperature
float32 afe3_bot_temp              # AFE 3 bottom temperature

# Shutdown Temperature Thresholds (Celsius)
float32 pcb_shutdown_temp          # PCB shutdown threshold
float32 afe0_top_shutdown_temp     # AFE 0 top shutdown threshold
float32 afe0_bot_shutdown_temp     # AFE 0 bottom shutdown threshold
float32 afe1_top_shutdown_temp     # AFE 1 top shutdown threshold
float32 afe1_bot_shutdown_temp     # AFE 1 bottom shutdown threshold
float32 afe2_top_shutdown_temp     # AFE 2 top shutdown threshold
float32 afe2_bot_shutdown_temp     # AFE 2 bottom shutdown threshold
float32 afe3_top_shutdown_temp     # AFE 3 top shutdown threshold
float32 afe3_bot_shutdown_temp     # AFE 3 bottom shutdown threshold

# Status Information
uint32 bootsts_register            # Boot status register
uint16 dip_switch                  # DIP switch settings

# Health Summary (computed)
bool healthy                       # Overall health status (all temps OK)
string health_message              # Human-readable health status message
