# API Usage

If you have an API for your package it is probably best to give some high-level instructions here rather than just relying on the automatically generated documentation.

- Give an overview of how you intend the api to be used. 
- Give a few examples
- Most importantly!   Give some "hello world" examples!

## Messages

### LoggerStatus.msg

The `LoggerStatus` message corresponds to the Svs5 message type 4 (LOGGER_REC_UPDATE) from the SequencerAPI, which represents the GLF::SOutputFileInfo structure.

**Fields:**
- `file_name` (string): Current recording file name
- `number_of_records` (uint32): Number of records in the file
- `file_size_bytes` (uint64): Current file size in bytes
- `disk_space_free_bytes` (uint64): Free disk space available in bytes
- `percent_disk_space_free` (float32): Percentage of disk space that is free
- `recording_time_left_secs` (uint32): Estimated recording time left in seconds

**Usage Example:**

```cpp
#include <template_pkg_interfaces/msg/logger_status.hpp>

// Callback for receiving logger status updates
void loggerStatusCallback(const template_pkg_interfaces::msg::LoggerStatus::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "Record Info:");
    RCLCPP_INFO(node->get_logger(), "  FileName: %s", msg->file_name.c_str());
    RCLCPP_INFO(node->get_logger(), "  No Of Records: %u", msg->number_of_records);
    RCLCPP_INFO(node->get_logger(), "  File Size (bytes): %lu", msg->file_size_bytes);
    RCLCPP_INFO(node->get_logger(), "  Free Disk Space: %lu", msg->disk_space_free_bytes);
    RCLCPP_INFO(node->get_logger(), "  Percentage Disk Space Free: %.2f", msg->percent_disk_space_free);
    RCLCPP_INFO(node->get_logger(), "  Recording Time Left: %u secs", msg->recording_time_left_secs);
}

// Create subscription
auto subscription = node->create_subscription<template_pkg_interfaces::msg::LoggerStatus>(
    "logger_status", 10, loggerStatusCallback);
```

**Publishing Example:**

```cpp
#include <template_pkg_interfaces/msg/logger_status.hpp>

// Create publisher
auto publisher = node->create_publisher<template_pkg_interfaces::msg::LoggerStatus>("logger_status", 10);

// Populate and publish message
auto msg = template_pkg_interfaces::msg::LoggerStatus();
msg.file_name = "recording_001.glf";
msg.number_of_records = 1234;
msg.file_size_bytes = 1024000;
msg.disk_space_free_bytes = 50000000000;
msg.percent_disk_space_free = 75.5;
msg.recording_time_left_secs = 3600;

publisher->publish(msg);
```
