#include "gemini_sonar_node.hpp"
#include <rclcpp/time.hpp>

NS_HEAD

// Initialize static instance pointer
GeminiSonarNode* GeminiSonarNode::instance_ = nullptr;

//=============================================================================
// Parameters Implementation
//=============================================================================

GeminiSonarNode::Parameters::Parameters() {}

void GeminiSonarNode::Parameters::declare(GeminiSonarNode* node)
{
    // Network configuration
    node->declare_parameter("sonar_id", sonar_id);
    node->declare_parameter("software_mode", software_mode);
    
    // Sonar operation parameters
    node->declare_parameter("range_m", range_m);
    node->declare_parameter("gain_percent", gain_percent);
    node->declare_parameter("sound_speed_ms", sound_speed_ms);
    node->declare_parameter("frequency_khz", frequency_khz);
    
    // Image/data parameters
    node->declare_parameter("num_beams", num_beams);
    node->declare_parameter("bins_per_beam", bins_per_beam);
    node->declare_parameter("beam_spacing_deg", beam_spacing_deg);
    
    // Topic configuration
    node->declare_parameter("topics.raw_sonar_image", topics.raw_sonar_image);
    node->declare_parameter("topics.projected_sonar_image", topics.projected_sonar_image);
    node->declare_parameter("topics.sonar_detections", topics.sonar_detections);
    node->declare_parameter("topics.raw_packet", topics.raw_packet);
}

void GeminiSonarNode::Parameters::update(GeminiSonarNode* node)
{
    node->get_parameter("sonar_id", sonar_id);
    node->get_parameter("software_mode", software_mode);
    
    node->get_parameter("range_m", range_m);
    node->get_parameter("gain_percent", gain_percent);
    node->get_parameter("sound_speed_ms", sound_speed_ms);
    node->get_parameter("frequency_khz", frequency_khz);
    
    node->get_parameter("num_beams", num_beams);
    node->get_parameter("bins_per_beam", bins_per_beam);
    node->get_parameter("beam_spacing_deg", beam_spacing_deg);
    
    node->get_parameter("topics.raw_sonar_image", topics.raw_sonar_image);
    node->get_parameter("topics.projected_sonar_image", topics.projected_sonar_image);
    node->get_parameter("topics.sonar_detections", topics.sonar_detections);
    node->get_parameter("topics.raw_packet", topics.raw_packet);
}

//=============================================================================
// Publishers Implementation
//=============================================================================

void GeminiSonarNode::Publishers::init(GeminiSonarNode* node)
{
    raw_sonar_image_ = node->create_publisher<marine_acoustic_msgs::msg::RawSonarImage>(
        node->parameters_.topics.raw_sonar_image, 10);
    
    projected_sonar_image_ = node->create_publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>(
        node->parameters_.topics.projected_sonar_image, 10);
    
    sonar_detections_ = node->create_publisher<marine_acoustic_msgs::msg::SonarDetections>(
        node->parameters_.topics.sonar_detections, 10);
    
    raw_packet_ = node->create_publisher<gemini_sonar_driver_interfaces::msg::RawPacket>(
        node->parameters_.topics.raw_packet, 10);
}

//=============================================================================
// Services Implementation
//=============================================================================

void GeminiSonarNode::Services::init(GeminiSonarNode* node)
{
    start_sonar_ = node->create_service<gemini_sonar_driver_interfaces::srv::StartSonar>(
        "gemini/start_sonar",
        std::bind(&GeminiSonarNode::handleStartSonar, node,
                  std::placeholders::_1, std::placeholders::_2));
    
    stop_sonar_ = node->create_service<gemini_sonar_driver_interfaces::srv::StopSonar>(
        "gemini/stop_sonar",
        std::bind(&GeminiSonarNode::handleStopSonar, node,
                  std::placeholders::_1, std::placeholders::_2));
}

//=============================================================================
// Constructor/Destructor
//=============================================================================

GeminiSonarNode::GeminiSonarNode()
    : Node("gemini_sonar_driver")
{
    RCLCPP_INFO(this->get_logger(), "Initializing Gemini Sonar Driver");
    
    // Set static instance for SDK callback
    instance_ = this;
    
    // Declare and read parameters
    parameters_.declare(this);
    parameters_.update(this);
    
    // Initialize conversion parameters from node parameters
    conversion_params_.frequency_khz = parameters_.frequency_khz;
    conversion_params_.sound_speed_ms = parameters_.sound_speed_ms;
    conversion_params_.range_m = parameters_.range_m;
    conversion_params_.num_beams = parameters_.num_beams;
    conversion_params_.bins_per_beam = parameters_.bins_per_beam;
    conversion_params_.beam_spacing_deg = parameters_.beam_spacing_deg;
    conversion_params_.frame_id = "gemini";
    
    // Initialize publishers and services
    publishers_.init(this);
    services_.init(this);
    
    // Initialize SDK
    if (initializeGeminiSDK())
    {
        RCLCPP_INFO(this->get_logger(), "Gemini SDK initialized successfully");
        
        // Configure sonar with parameters
        if (configureSonar())
        {
            RCLCPP_INFO(this->get_logger(), "Sonar configured successfully");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure sonar");
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Gemini SDK");
    }
    
    RCLCPP_INFO(this->get_logger(), "Gemini Sonar Driver ready. Use services to start/stop sonar.");
}

GeminiSonarNode::~GeminiSonarNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Gemini Sonar Driver");
    
    if (sonar_running_)
    {
        stopPinging();
    }
    
    shutdownGeminiSDK();
    
    instance_ = nullptr;
}

//=============================================================================
// Service Handlers
//=============================================================================

void GeminiSonarNode::handleStartSonar(
    const std::shared_ptr<gemini_sonar_driver_interfaces::srv::StartSonar::Request> request,
    std::shared_ptr<gemini_sonar_driver_interfaces::srv::StartSonar::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received start sonar request");
    
    if (sonar_running_)
    {
        response->success = false;
        response->message = "Sonar is already running";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    // Start pinging
    if (startPinging())
    {
        response->success = true;
        response->message = "Sonar started successfully";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
    else
    {
        response->success = false;
        response->message = "Failed to start sonar";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
}

void GeminiSonarNode::handleStopSonar(
    const std::shared_ptr<gemini_sonar_driver_interfaces::srv::StopSonar::Request> request,
    std::shared_ptr<gemini_sonar_driver_interfaces::srv::StopSonar::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received stop sonar request");
    
    if (!sonar_running_)
    {
        response->success = false;
        response->message = "Sonar is not running";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
    if (stopPinging())
    {
        response->success = true;
        response->message = "Sonar stopped successfully";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
    else
    {
        response->success = false;
        response->message = "Failed to stop sonar";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
}

//=============================================================================
// Gemini SDK Callbacks
//=============================================================================

void GeminiSonarNode::geminiDataCallback(int messageType, int length, char* dataBlock)
{
    if (instance_ != nullptr)
    {
        instance_->processGeminiData(messageType, length, dataBlock);
    }
}

void GeminiSonarNode::processGeminiData(int messageType, int length, char* dataBlock)
{
    // Publish raw packet
    auto raw_msg = std::make_shared<gemini_sonar_driver_interfaces::msg::RawPacket>();
    raw_msg->header.stamp = this->now();
    raw_msg->header.frame_id = "gemini";
    raw_msg->message_type = messageType;
    raw_msg->data.assign(dataBlock, dataBlock + length);
    publishers_.raw_packet_->publish(*raw_msg);
    
    // Process based on message type
    switch (messageType)
    {
        case 1:  // Ping Head
            processPingHead(dataBlock, length);
            break;
            
        case 2:  // Ping Line (beam data)
            processPingLine(dataBlock, length);
            break;
            
        case 3:  // Ping Tail
            processPingTail(dataBlock, length);
            break;
            
        default:
            RCLCPP_DEBUG(this->get_logger(), "Received unknown message type: %d", messageType);
            break;
    }
}

void GeminiSonarNode::processPingHead(const char* data, int length)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Reset ping data
    current_ping_beams_.clear();
    ping_complete_ = false;
    
    // Parse ping head structure from Gemini SDK
    const CGemPingHead* ping_head = reinterpret_cast<const CGemPingHead*>(data);
    
    // Extract ping metadata
    ping_number_ = ping_head->m_pingID;
    
    // Calculate actual range from start/end range fields
    // These are typically in units of sound speed bins
    range_m_ = (ping_head->m_endRange - ping_head->m_startRange) * 
               (ping_head->m_sosUsed / 2000.0);  // Convert to meters
    
    // Extract timestamp (64-bit timestamp from two 32-bit fields)
    uint64_t timestamp_us = (static_cast<uint64_t>(ping_head->m_transmitTimestampH) << 32) | 
                            ping_head->m_transmitTimestampL;
    ping_time_ = timestamp_us / 1000000.0;  // Convert to seconds
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "Ping head: ID=%d, Range=%.1fm, Beams=%d, SOS=%d m/s",
                 ping_number_, range_m_, ping_head->m_numBeams, ping_head->m_sosUsed);
}

void GeminiSonarNode::processPingLine(const char* data, int length)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Parse ping line structure from Gemini SDK
    // Each line represents one beam's intensity data
    const CGemPingLine* ping_line = reinterpret_cast<const CGemPingLine*>(data);
    
    // Get the actual data width (number of samples in this beam)
    int line_width = ping_line->GetLineWidth();
    
    // Extract the beam data (starts after the CGemPingLine header)
    const uint8_t* beam_data_ptr = reinterpret_cast<const uint8_t*>(&ping_line->m_startOfData);
    
    // Store beam data as vector
    std::vector<uint8_t> beam_data(beam_data_ptr, beam_data_ptr + line_width);
    current_ping_beams_.push_back(beam_data);
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "Received beam %zu: line_id=%d, width=%d samples, gain=%d, scale=%d",
                 current_ping_beams_.size(), ping_line->m_lineID, 
                 line_width, ping_line->m_gain, ping_line->m_scale);
}

void GeminiSonarNode::processPingTail(const char* data, int length)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    ping_complete_ = true;
    ping_number_++;
    
    RCLCPP_DEBUG(this->get_logger(), "Received ping tail - ping complete");
    
    // Check if we have beam data to publish
    if (current_ping_beams_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Ping complete but no beam data available");
        return;
    }
    
    // Use conversions module to create messages
    rclcpp::Time timestamp = this->now();
    
    // Publish complete ping as RawSonarImage
    auto raw_msg = conversions::createRawSonarImage(current_ping_beams_, conversion_params_, timestamp);
    if (raw_msg)
    {
        publishers_.raw_sonar_image_->publish(*raw_msg);
    }
    
    // Publish SonarDetections (best for FLS data analysis)
    auto detections_msg = conversions::createSonarDetections(current_ping_beams_, conversion_params_, timestamp);
    if (detections_msg)
    {
        publishers_.sonar_detections_->publish(*detections_msg);
    }
    
    // Publish projected sonar image periodically (every 10 pings)
    if (ping_number_ % 10 == 0)
    {
        auto proj_msg = conversions::createProjectedSonarImage(current_ping_beams_, conversion_params_, timestamp);
        if (proj_msg)
        {
            publishers_.projected_sonar_image_->publish(*proj_msg);
        }
    }
}

//=============================================================================
// SDK Initialization and Configuration
//=============================================================================

bool GeminiSonarNode::initializeGeminiSDK()
{
    RCLCPP_INFO(this->get_logger(), "Initializing Gemini SDK...");
    
    // Set software mode
    GEM_SetGeminiSoftwareMode(parameters_.software_mode.c_str());
    
    // Set callback function
    GEM_SetHandlerFunction(&GeminiSonarNode::geminiDataCallback);
    
    // Start network interface
    int result = GEM_StartGeminiNetworkWithResult(parameters_.sonar_id, false);
    
    if (result == 1)
    {
        sdk_initialized_ = true;
        return true;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), 
                     "Failed to initialize Gemini network. Another program may be using it.");
        return false;
    }
}

bool GeminiSonarNode::configureSonar()
{
    if (!sdk_initialized_)
    {
        RCLCPP_ERROR(this->get_logger(), "SDK not initialized");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Configuring sonar parameters:");
    RCLCPP_INFO(this->get_logger(), "  Range: %.1f m", parameters_.range_m);
    RCLCPP_INFO(this->get_logger(), "  Gain: %.1f %%", parameters_.gain_percent);
    RCLCPP_INFO(this->get_logger(), "  Sound Speed: %d m/s", parameters_.sound_speed_ms);
    RCLCPP_INFO(this->get_logger(), "  Frequency: %.1f kHz", parameters_.frequency_khz);
    RCLCPP_INFO(this->get_logger(), "  Number of Beams: %d", parameters_.num_beams);
    
    // Configure ping using AutoPingConfig (sets range, gain, and speed of sound)
    GEMX_AutoPingConfig(
        parameters_.sonar_id,
        static_cast<float>(parameters_.range_m),
        static_cast<unsigned short>(parameters_.gain_percent),
        static_cast<float>(parameters_.sound_speed_ms)
    );
    
    // Set number of beams (256 or 512 typically)
    GEMX_SetGeminiBeams(parameters_.sonar_id, static_cast<unsigned short>(parameters_.num_beams));
    
    // Send the ping configuration to the sonar
    GEMX_SendGeminiPingConfig(parameters_.sonar_id);
    
    RCLCPP_INFO(this->get_logger(), "Sonar configuration sent successfully");
    return true;
}

bool GeminiSonarNode::startPinging()
{
    if (!sdk_initialized_)
    {
        RCLCPP_ERROR(this->get_logger(), "SDK not initialized");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Starting sonar pinging...");
    
    // Set ping mode to continuous pinging (mode 1)
    // Mode 0: Single ping on receipt of config
    // Mode 1: Continuous pinging at inter-ping period interval
    GEMX_SetPingMode(parameters_.sonar_id, 1);
    
    // Set inter-ping period (default to 100ms = 100000 microseconds)
    // This controls how fast the sonar pings
    GEMX_SetInterPingPeriod(parameters_.sonar_id, 100000);
    
    // Send ping configuration to actually start pinging
    GEMX_SendGeminiPingConfig(parameters_.sonar_id);
    
    sonar_running_ = true;
    RCLCPP_INFO(this->get_logger(), "Sonar pinging started");
    return true;
}

bool GeminiSonarNode::stopPinging()
{
    RCLCPP_INFO(this->get_logger(), "Stopping sonar pinging...");
    
    // Set ping mode to single ping (mode 0) to stop continuous pinging
    GEMX_SetPingMode(parameters_.sonar_id, 0);
    
    sonar_running_ = false;
    RCLCPP_INFO(this->get_logger(), "Sonar pinging stopped");
    return true;
}

void GeminiSonarNode::shutdownGeminiSDK()
{
    if (sdk_initialized_)
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down Gemini SDK");
        
        // Stop the Gemini network interface
        GEM_StopGeminiNetwork();
        
        sdk_initialized_ = false;
        RCLCPP_INFO(this->get_logger(), "Gemini SDK shutdown complete");
    }
}

NS_FOOT

