std_msgs/Header header

# String representation of sonar IP address (e.g., 192.168.2.10)
string ip_address

# Numerical sonar/device identifier reported by the Gemini
uint32 sonar_id

# Status flags derived from shutdown/boot status registers
bool bootloader_mode
bool over_temperature
bool out_of_water

# Raw status register values for debugging
uint16 shutdown_status
uint16 boot_status
