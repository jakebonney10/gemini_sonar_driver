#pragma once

// Local package includes
#include "package_defs.hpp"

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/raw_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>
#include <marine_acoustic_msgs/msg/sonar_detections.hpp>
#include <marine_acoustic_msgs/msg/detection_flag.hpp>
#include <gemini_sonar_driver_interfaces/msg/raw_packet.hpp>
#include <gemini_sonar_driver_interfaces/srv/start_sonar.hpp>
#include <gemini_sonar_driver_interfaces/srv/stop_sonar.hpp>

// Standard library
#include <memory>
#include <string>
#include <fstream>
#include <mutex>
#include <atomic>
#include <vector>

// Define cdecl as empty on non-Windows platforms (it's the default calling convention on Linux)
#ifndef _WIN32
#ifndef cdecl
#define cdecl
#endif
#endif

// Gemini SDK includes (order matters: structures must come before comms)
#include "Gemini/GeminiStructuresPublic.h"
#include "Gemini/GeminiCommsPublic.h"

NS_HEAD

/**
 * @brief Main driver node for Tritech Gemini 1200ikd multibeam sonar
 * 
 * This node interfaces with the Gemini SDK to:
 * - Configure sonar parameters via ROS2 parameters
 * - Start/stop sonar operation via ROS2 services
 * - Publish multibeam data using marine_acoustic_msgs (RawSonarImage, ProjectedSonarImage)
 * - Log data in native Gemini format alongside ROS2 bags
 */
class GeminiSonarNode : public rclcpp::Node
{
public:
    GeminiSonarNode();
    ~GeminiSonarNode();

    /**
     * @brief Sonar configuration parameters
     */
    struct Parameters
    {
        // Network configuration
        uint16_t sonar_id = 1;                           ///< Sonar ID (default 1)
        std::string software_mode = "Evo";                ///< SDK mode: Evo, EvoC, SeaNet, SeaNetC
        
        // Sonar operation parameters
        double range_m = 75.0;                            ///< Range in meters
        double gain_percent = 50.0;                       ///< Receiver gain (0-100%)
        int sound_speed_ms = 1500;                        ///< Sound speed in m/s
        double frequency_khz = 720.0;                     ///< Operating frequency in kHz
        
        // Image/data parameters
        int num_beams = 512;                              ///< Number of beams
        int bins_per_beam = 1500;                         ///< Bins per beam (range cells)
        double beam_spacing_deg = 0.25;                   ///< Beam spacing in degrees
        
        // Logging parameters
        bool enable_native_logging = true;                ///< Enable Gemini native format logging
        std::string native_log_directory = "";            ///< Directory for native logs
        
        // Topic configuration
        struct Topics
        {
            std::string raw_sonar_image = "gemini/raw_sonar_image";           ///< marine_acoustic_msgs/RawSonarImage
            std::string projected_sonar_image = "gemini/projected_sonar";     ///< marine_acoustic_msgs/ProjectedSonarImage
            std::string sonar_detections = "gemini/detections";               ///< marine_acoustic_msgs/SonarDetections
            std::string raw_packet = "gemini/raw";                            ///< Raw Gemini packets
        } topics;

        Parameters();
        void declare(GeminiSonarNode* node);
        void update(GeminiSonarNode* node);
    };

    /**
     * @brief ROS2 publishers
     */
    struct Publishers
    {
        rclcpp::Publisher<marine_acoustic_msgs::msg::RawSonarImage>::SharedPtr raw_sonar_image_;
        rclcpp::Publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr projected_sonar_image_;
        rclcpp::Publisher<marine_acoustic_msgs::msg::SonarDetections>::SharedPtr sonar_detections_;
        rclcpp::Publisher<gemini_sonar_driver_interfaces::msg::RawPacket>::SharedPtr raw_packet_;
        
        void init(GeminiSonarNode* node);
    };

    /**
     * @brief ROS2 services
     */
    struct Services
    {
        rclcpp::Service<gemini_sonar_driver_interfaces::srv::StartSonar>::SharedPtr start_sonar_;
        rclcpp::Service<gemini_sonar_driver_interfaces::srv::StopSonar>::SharedPtr stop_sonar_;
        
        void init(GeminiSonarNode* node);
    };

protected:
    /**
     * @brief Service callback to start the sonar
     */
    void handleStartSonar(
        const std::shared_ptr<gemini_sonar_driver_interfaces::srv::StartSonar::Request> request,
        std::shared_ptr<gemini_sonar_driver_interfaces::srv::StartSonar::Response> response);

    /**
     * @brief Service callback to stop the sonar
     */
    void handleStopSonar(
        const std::shared_ptr<gemini_sonar_driver_interfaces::srv::StopSonar::Request> request,
        std::shared_ptr<gemini_sonar_driver_interfaces::srv::StopSonar::Response> response);

    /**
     * @brief Static callback function for Gemini SDK data
     * This is called by the SDK when data is received
     */
    static void geminiDataCallback(int messageType, int length, char* dataBlock);

    /**
     * @brief Instance method to process Gemini data
     */
    void processGeminiData(int messageType, int length, char* dataBlock);

    /**
     * @brief Process ping head message
     */
    void processPingHead(const char* data, int length);

    /**
     * @brief Process ping line message (beam data)
     */
    void processPingLine(const char* data, int length);

    /**
     * @brief Process ping tail message
     */
    void processPingTail(const char* data, int length);

    /**
     * @brief Initialize the Gemini SDK and configure sonar
     */
    bool initializeGeminiSDK();

    /**
     * @brief Configure sonar parameters via SDK
     */
    bool configureSonar();

    /**
     * @brief Start the sonar pinging
     */
    bool startPinging();

    /**
     * @brief Stop the sonar pinging
     */
    bool stopPinging();

    /**
     * @brief Shutdown the Gemini SDK
     */
    void shutdownGeminiSDK();

    /**
     * @brief Open native format log file
     */
    bool openNativeLog(const std::string& directory);

    /**
     * @brief Close native format log file
     */
    void closeNativeLog();

    /**
     * @brief Write data to native format log
     */
    void writeToNativeLog(const char* data, int length, int messageType);

    /**
     * @brief Convert Gemini data to marine_acoustic_msgs/RawSonarImage
     */
    marine_acoustic_msgs::msg::RawSonarImage::SharedPtr createRawSonarImageMsg();

    /**
     * @brief Convert Gemini data to marine_acoustic_msgs/ProjectedSonarImage
     */
    marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr createProjectedSonarImageMsg();

    /**
     * @brief Convert Gemini data to marine_acoustic_msgs/SonarDetections
     */
    marine_acoustic_msgs::msg::SonarDetections::SharedPtr createSonarDetectionsMsg();

    // Member variables
    Parameters parameters_;
    Publishers publishers_;
    Services services_;

    // SDK state
    std::atomic<bool> sonar_running_{false};
    std::atomic<bool> sdk_initialized_{false};
    
    // Data buffers (protected by mutex)
    std::mutex data_mutex_;
    std::vector<std::vector<uint8_t>> current_ping_beams_;  ///< Current ping beam data
    bool ping_complete_{false};
    
    // Ping head data
    uint32_t ping_number_{0};
    double ping_time_{0.0};
    double range_m_{0.0};
    
    // Native logging
    std::ofstream native_log_file_;
    std::mutex log_mutex_;
    bool native_logging_enabled_{false};

    // Static instance pointer for SDK callback
    static GeminiSonarNode* instance_;
};

NS_FOOT
