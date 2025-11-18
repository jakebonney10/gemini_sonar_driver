# Logger recording status information from LOGGER_REC_UPDATE message
std_msgs/Header header

string file_name                # Current recording file name
uint32 number_of_records        # Number of records in the current file
uint64 file_size_bytes          # Current file size in bytes
uint64 disk_space_free_bytes    # Free disk space in bytes
float32 percent_disk_space_free # Percentage of disk space free (0-100)
uint32 recording_time_left_secs # Recording time left in seconds

