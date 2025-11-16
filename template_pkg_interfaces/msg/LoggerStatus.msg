# Logger recording update information
# Corresponds to GLF::SOutputFileInfo from SequencerAPI
# Message type 4: LOGGER_REC_UPDATE

string file_name                        # m_strFileName: Current recording file name
uint32 number_of_records                # m_uiNumberOfRecords: Number of records in file
uint64 file_size_bytes                  # m_fileSizeBytes: Current file size in bytes
uint64 disk_space_free_bytes            # m_diskSpaceFreeBytes: Free disk space in bytes
float32 percent_disk_space_free         # m_percentDiskSpaceFree: Percentage of disk space free
uint32 recording_time_left_secs         # m_recordingTimeLeftSecs: Estimated recording time left in seconds
