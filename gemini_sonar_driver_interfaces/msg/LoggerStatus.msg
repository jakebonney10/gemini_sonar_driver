# Logger recording status information from LOGGER_REC_UPDATE message
std_msgs/Header header

# Current recording file name
string file_name

# Number of records in the current file
uint32 number_of_records

# Current file size in bytes
uint64 file_size_bytes

# Free disk space in bytes
uint64 disk_space_free_bytes

# Percentage of disk space free (0-100)
float32 percent_disk_space_free

# Recording time left in seconds
uint32 recording_time_left_secs
