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
// Svs5Sequencer SDK Callbacks
//=============================================================================

void GeminiSonarNode::processSvs5Message(unsigned int messageType, unsigned int size, const char* const value)
{
    // Publish raw packet
    auto raw_msg = std::make_shared<gemini_sonar_driver_interfaces::msg::RawPacket>();
    raw_msg->header.stamp = this->now();
    raw_msg->header.frame_id = "gemini";
    raw_msg->message_type = messageType;
    raw_msg->data.assign(value, value + size);
    publishers_.raw_packet_->publish(*raw_msg);
    
    // Process based on Svs5 message type
    switch (static_cast<SequencerApi::ESvs5MessageType>(messageType))
    {
        case SequencerApi::GEMINI_STATUS:
            RCLCPP_DEBUG(this->get_logger(), "Received GEMINI_STATUS message");
            // Parse CGeminiStatusData structure for sonar health/status
            break;
            
        case SequencerApi::GLF_LIVE_TARGET_IMAGE:
            // This is the processed sonar image data (GLF format)
            RCLCPP_DEBUG(this->get_logger(), "Received GLF_LIVE_TARGET_IMAGE");
            // Parse GLF::CTargetImageData structure
            processPingData(value, size);
            break;
            
        case SequencerApi::SENSOR_RECORD:
            RCLCPP_DEBUG(this->get_logger(), "Received SENSOR_RECORD");
            // Parse sensor data (GPS, compass, etc.)
            break;
            
        case SequencerApi::FRAME_RATE:
            RCLCPP_DEBUG(this->get_logger(), "Received FRAME_RATE message");
            break;
            
        default:
            RCLCPP_DEBUG(this->get_logger(), "Received Svs5 message type: %u", messageType);
            break;
    }
}

void GeminiSonarNode::processPingData(const char* data, int length)
{
    // The Svs5Sequencer provides processed sonar image data in GLF format
    // This contains the full formed beams with metadata
    // TODO: Parse GLF::CTargetImageData structure from Svs5SequencerStructures.h
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // For now, use a simplified approach - extract beam data directly
    // In production, you should properly parse the GLF::CTargetImageData structure
    
    ping_number_++;
    rclcpp::Time timestamp = this->now();
    
    // Publish messages using the conversions module
    // Note: You'll need to properly parse the GLF format to extract beam data
    // This is a placeholder that shows the pattern
    
    RCLCPP_DEBUG(this->get_logger(), "Processing ping data (length=%d bytes)", length);
}

//=============================================================================
// SDK Initialization and Configuration
//=============================================================================

bool GeminiSonarNode::initializeGeminiSDK()
{
    RCLCPP_INFO(this->get_logger(), "Initializing Gemini SDK (Svs5Sequencer API)...");
    
    // Set the static instance pointer for the callback
    instance_ = this;
    
    // Create the callback lambda
    SequencerApi::Svs5Callback callback = [](unsigned int msgType, unsigned int size, const char* const value) {
        if (instance_) {
            instance_->processSvs5Message(msgType, size, value);
        }
    };
    
    // Start the Svs5 library - auto-discovers sonars on the network
    Svs5ErrorCode result = SequencerApi::StartSvs5(callback, false);
    
    if (result != SVS5_SEQUENCER_STATUS_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start Svs5 library. Error code: 0x%08lX", result);
        if (result == SVS5_SEQUENCER_ANOTHER_INSTANCE_RUNNING) {
            RCLCPP_ERROR(this->get_logger(), "  -> Another instance running (check for other Tritech software)");
        }
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Svs5 library started successfully");
    RCLCPP_INFO(this->get_logger(), "Library version: %s", SequencerApi::GetLibraryVersionInfo());
    
    sdk_initialized_ = true;
    return true;
}

bool GeminiSonarNode::configureSonar()
{
    if (!sdk_initialized_)
    {
        RCLCPP_ERROR(this->get_logger(), "SDK not initialized");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Configuring Gemini 1200ik sonar via Svs5Sequencer:");
    RCLCPP_INFO(this->get_logger(), "  Range: %.1f m", parameters_.range_m);
    RCLCPP_INFO(this->get_logger(), "  Gain: %.1f %%", parameters_.gain_percent);
    RCLCPP_INFO(this->get_logger(), "  Sound Speed: %d m/s", parameters_.sound_speed_ms);
    RCLCPP_INFO(this->get_logger(), "  Number of Beams: %d", parameters_.num_beams);
    
    Svs5ErrorCode result;
    
    // 1. Configure range
    double range = parameters_.range_m;
    result = SequencerApi::Svs5SetConfiguration(
        SequencerApi::SVS5_CONFIG_RANGE,
        sizeof(double),
        &range,
        parameters_.sonar_id
    );
    if (result != SVS5_SEQUENCER_STATUS_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set range");
        return false;
    }
    
    // 2. Configure gain
    int gain = static_cast<int>(parameters_.gain_percent);
    result = SequencerApi::Svs5SetConfiguration(
        SequencerApi::SVS5_CONFIG_GAIN,
        sizeof(int),
        &gain,
        parameters_.sonar_id
    );
    if (result != SVS5_SEQUENCER_STATUS_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set gain");
        return false;
    }
    
    // 3. Configure speed of sound (manual mode)
    SequencerApi::SequencerSosConfig sosConfig;
    sosConfig.m_bUsedUserSos = true;  // Use manual SOS
    sosConfig.m_manualSos = static_cast<float>(parameters_.sound_speed_ms);
    
    result = SequencerApi::Svs5SetConfiguration(
        SequencerApi::SVS5_CONFIG_SOUND_VELOCITY,
        sizeof(SequencerApi::SequencerSosConfig),
        &sosConfig,
        parameters_.sonar_id
    );
    if (result != SVS5_SEQUENCER_STATUS_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set sound velocity");
        return false;
    }
    
    // 4. Configure image quality (beams)
    SequencerApi::SonarImageQualityLevel qualityLevel;
    if (parameters_.num_beams >= 512) {
        qualityLevel.m_performance = SequencerApi::UL_HIGH_CPU;  // 512-1024 beams
    } else {
        qualityLevel.m_performance = SequencerApi::HIGH_CPU;      // 256-512 beams
    }
    qualityLevel.m_screenPixels = 2048;  // Highest quality
    
    result = SequencerApi::Svs5SetConfiguration(
        SequencerApi::SVS5_CONFIG_CPU_PERFORMANCE,
        sizeof(SequencerApi::SonarImageQualityLevel),
        &qualityLevel,
        parameters_.sonar_id
    );
    if (result != SVS5_SEQUENCER_STATUS_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to set performance level (non-critical)");
    }
    
    // 5. Enable high range resolution for 1200ik
    bool highRes = true;
    result = SequencerApi::Svs5SetConfiguration(
        SequencerApi::SVS5_CONFIG_HIGH_RESOLUTION,
        sizeof(bool),
        &highRes,
        parameters_.sonar_id
    );
    if (result != SVS5_SEQUENCER_STATUS_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to set high resolution (non-critical)");
    }
    
    // 6. Configure chirp mode (auto)
    int chirpMode = 2;  // 0=disabled, 1=enabled, 2=auto
    result = SequencerApi::Svs5SetConfiguration(
        SequencerApi::SVS5_CONFIG_CHIRP_MODE,
        sizeof(int),
        &chirpMode,
        parameters_.sonar_id
    );
    if (result != SVS5_SEQUENCER_STATUS_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to set chirp mode (non-critical)");
    }
    
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
    
    RCLCPP_INFO(this->get_logger(), "Starting sonar streaming...");
    
    // Configure ping mode (free-running or interval-based)
    SequencerApi::SequencerPingMode pingMode;
    pingMode.m_bFreeRun = false;      // Ping at fixed interval
    pingMode.m_msInterval = 100;      // 100ms between pings
    pingMode.m_extTTLTrigger = false; // No external trigger
    
    Svs5ErrorCode result = SequencerApi::Svs5SetConfiguration(
        SequencerApi::SVS5_CONFIG_PING_MODE,
        sizeof(SequencerApi::SequencerPingMode),
        &pingMode,
        parameters_.sonar_id
    );
    
    if (result != SVS5_SEQUENCER_STATUS_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set ping mode");
        return false;
    }
    
    // Start streaming (set online mode)
    bool online = true;
    result = SequencerApi::Svs5SetConfiguration(
        SequencerApi::SVS5_CONFIG_ONLINE,
        sizeof(bool),
        &online,
        parameters_.sonar_id
    );
    
    if (result != SVS5_SEQUENCER_STATUS_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start sonar streaming");
        return false;
    }
    
    sonar_running_ = true;
    RCLCPP_INFO(this->get_logger(), "Sonar streaming started");
    return true;
}

bool GeminiSonarNode::stopPinging()
{
    RCLCPP_INFO(this->get_logger(), "Stopping sonar streaming...");
    
    // Stop streaming (set offline mode)
    bool online = false;
    Svs5ErrorCode result = SequencerApi::Svs5SetConfiguration(
        SequencerApi::SVS5_CONFIG_ONLINE,
        sizeof(bool),
        &online,
        parameters_.sonar_id
    );
    
    if (result != SVS5_SEQUENCER_STATUS_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop sonar streaming");
        return false;
    }
    
    sonar_running_ = false;
    RCLCPP_INFO(this->get_logger(), "Sonar streaming stopped");
    return true;
}

void GeminiSonarNode::shutdownGeminiSDK()
{
    if (sdk_initialized_)
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down Gemini SDK");
        
        // Stop the Svs5 library
        SequencerApi::StopSvs5();
        
        sdk_initialized_ = false;
        RCLCPP_INFO(this->get_logger(), "Gemini SDK shutdown complete");
    }
}

NS_FOOT

