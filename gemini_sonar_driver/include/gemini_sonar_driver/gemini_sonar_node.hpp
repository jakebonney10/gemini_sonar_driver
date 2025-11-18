#pragma once

// Local package includes
#include "package_defs.hpp"
#include "gemini_sonar_driver/conversions.hpp"

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/raw_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>
#include <marine_acoustic_msgs/msg/sonar_detections.hpp>
#include <marine_acoustic_msgs/msg/detection_flag.hpp>
#include <gemini_sonar_driver_interfaces/msg/raw_packet.hpp>
#include <gemini_sonar_driver_interfaces/msg/gemini_status.hpp>
#include <gemini_sonar_driver_interfaces/msg/logger_status.hpp>
#include <gemini_sonar_driver_interfaces/srv/start_sonar.hpp>
#include <gemini_sonar_driver_interfaces/srv/stop_sonar.hpp>

// Standard library
#include <memory>
#include <string>
#include <fstream>
#include <mutex>
#include <atomic>
#include <vector>
#include <chrono>
#include <thread>

// Define cdecl as empty on non-Windows platforms (it's the default calling convention on Linux)
#ifndef _WIN32
#ifndef cdecl
#define cdecl
#endif
#endif

// Gemini SDK Svs5Sequencer API (high-level interface)
#include "types.h"  // UInt8, UInt16, UInt32 typedefs
#include "Svs5Seq/Svs5SequencerApi.h"
#include "Gemini/GeminiStructuresPublic.h"
#include "GenesisSerializer/GlfLoggerGeminiStructure.h"
#include "GenesisSerializer/GeminiStatusRecord.h"  // For GLF::GeminiSonarStatusMessage and GLF::GeminiStatusRecord
#include "GenesisSerializer/GlfLoggerStatusStructure.h"  // For GLF::SOutputFileInfo

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
        // Frame configuration
        std::string frame_id = "gemini_fls";              ///< TF frame ID for sonar data

        // Logging configuration
        std::string log_directory = "/data/gemini";        ///< Directory to save GLF log files

        // Network configuration
        uint16_t sonar_id = 0;                            ///< Sonar ID (default 1, 0 will reach all sonars on network)
        std::string software_mode = "Evo";                ///< SDK mode: Evo, EvoC, SeaNet, SeaNetC
        
        // Sonar operation 
        double range_m = 10.0;                            ///< Range in meters (1-120m for 720kHz, 1-50m for 1200kHz)
        double gain_percent = 50.0;                       ///< Receiver gain (0-100%)
        double aperture = 120.0;                          ///< Sonar aperture in degrees (Switch between: 120 or 65)
        int sound_speed_ms = 1500;                        ///< Sound speed in m/s
        bool sound_speed_manual = false;                  ///< Sound speed mode: true=manual, false=auto (uses sonar SOS sensor)
        int chirp_mode = 2;                               ///< Chirp mode: 0=disabled, 1=enabled, 2=auto
        bool high_resolution = true;                      ///< High resolution mode (1200ik only): true=1200kHz, false=720kHz
        
        // Ping mode
        bool ping_free_run = false;                       ///< Ping mode: true=continuous, false=interval-based
        int ping_interval_ms = 100;                       ///< Ping interval in ms (0-999) when free_run=false
        bool ping_ext_trigger = false;                    ///< External TTL trigger: true=hardware trigger, false=software

        struct Topics
        {
            std::string raw_sonar_image = "gemini/raw_sonar_image";           ///< marine_acoustic_msgs/RawSonarImage
            std::string projected_sonar_image = "gemini/projected_sonar";     ///< marine_acoustic_msgs/ProjectedSonarImage
            std::string sonar_detections = "gemini/detections";               ///< marine_acoustic_msgs/SonarDetections
            std::string raw_packet = "gemini/raw";                            ///< Raw Gemini packets
            std::string status = "gemini/status";                             ///< Gemini device status
            std::string logger_status = "gemini/logger_status";               ///< Logger recording status
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
        rclcpp::Publisher<gemini_sonar_driver_interfaces::msg::GeminiStatus>::SharedPtr status_;
        rclcpp::Publisher<gemini_sonar_driver_interfaces::msg::LoggerStatus>::SharedPtr logger_status_;
        
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
     * @brief Static callback function for Svs5Sequencer API
     * This is called by the SDK when data is received
     */
    static void svs5DataCallback(unsigned int messageType, unsigned int size, const char* const value);

    /**
     * @brief Instance method to handle Svs5 messages
     */
    void handleSvs5Message(unsigned int messageType, unsigned int size, const char* const value);

    /**
     * @brief Process GLF status messages
     */
    void processGeminiStatus(const GLF::GeminiStatusRecord* pStatus);

    /**
     * @brief Process GLF sonar image data
     */
    void processGLFImage(const GLF::GLogTargetImage& image);

    /**
     * @brief Process logger recording update messages
     */
    void processLoggerRecUpdate(const GLF::SOutputFileInfo* loggerInfo);

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
     * @brief Start Gemini data logging (Native GLF format)
     */
    void startLogging(std::string log_directory);

    /**
     * @brief Stop Gemini data logging (Native GLF format)
     */
    void stopLogging();

    /**
     * @brief Shutdown the Gemini SDK
     */
    void shutdownGeminiSDK();

    /**
     * @brief Wait for sonar to be detected on network
     * @param timeout_seconds Maximum time to wait in seconds
     * @return true if sonar detected, false if timeout
     */
    bool waitForSonarDetection(int timeout_seconds);

    /**
     * @brief Helper to set SDK configuration with error logging
     * @param config_type The configuration type to set
     * @param size Size of the configuration data
     * @param data Pointer to configuration data
     * @param param_name Human-readable parameter name for logging
     */
    void setSdkParameter(SequencerApi::ESvs5ConfigType config_type, 
                         size_t size, 
                         const void* data, 
                         const std::string& param_name);

    // Member variables
    Parameters parameters_;
    Publishers publishers_;
    Services services_;

    // Conversion parameters for message creation
    conversions::ConversionParameters conversion_params_;

    // SDK state
    std::atomic<bool> sonar_streaming_{false};
    std::atomic<bool> sdk_initialized_{false};
    std::atomic<bool> sonar_detected_{false};        ///< True if we've received any messages from sonar
    std::atomic<uint64_t> last_message_time_{0};     ///< Timestamp of last received message
    
    // Data buffers (protected by mutex)
    std::mutex data_mutex_;
    std::vector<std::vector<uint8_t>> current_ping_beams_;  ///< Current ping beam data
    bool ping_complete_{false};
    
    // Ping head data
    uint32_t ping_number_{0};
    double ping_time_{0.0};
    double range_m_{0.0};

    // Static instance pointer for SDK callback
    static GeminiSonarNode* instance_;
};

NS_FOOT
