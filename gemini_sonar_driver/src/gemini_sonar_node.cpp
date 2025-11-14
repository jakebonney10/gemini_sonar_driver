#include "gemini_sonar_node.hpp"
#include <rclcpp/time.hpp>
#include <sstream>

NS_HEAD

// Initialize static instance pointer
GeminiSonarNode* GeminiSonarNode::instance_ = nullptr;

//=============================================================================
// Parameters Implementation
//=============================================================================

GeminiSonarNode::Parameters::Parameters() {}

void GeminiSonarNode::Parameters::declare(GeminiSonarNode* node)
{
    // Network config
    node->declare_parameter("sonar_id", sonar_id);
    node->declare_parameter("software_mode", software_mode);
    
    // Sonar operation 
    node->declare_parameter("range_m", range_m);
    node->declare_parameter("gain_percent", gain_percent);
    node->declare_parameter("sound_speed_ms", sound_speed_ms);
    node->declare_parameter("frequency_khz", frequency_khz);
    
    // Image/data 
    node->declare_parameter("num_beams", num_beams);
    node->declare_parameter("bins_per_beam", bins_per_beam);
    
    // Frame ID
    node->declare_parameter("frame_id", frame_id);
    
    // Advanced sonar 
    node->declare_parameter("chirp_mode", chirp_mode);
    node->declare_parameter("high_resolution", high_resolution);
    
    // Ping mode 
    node->declare_parameter("ping_free_run", ping_free_run);
    node->declare_parameter("ping_interval_ms", ping_interval_ms);
    node->declare_parameter("ping_ext_trigger", ping_ext_trigger);
    
    // Topic names
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
    
    node->get_parameter("frame_id", frame_id);
    
    node->get_parameter("chirp_mode", chirp_mode);
    node->get_parameter("high_resolution", high_resolution);
    
    node->get_parameter("ping_free_run", ping_free_run);
    node->get_parameter("ping_interval_ms", ping_interval_ms);
    node->get_parameter("ping_ext_trigger", ping_ext_trigger);
    
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
    
    // Declare and read parameters
    parameters_.declare(this);
    parameters_.update(this);

    // Initialize publishers and services
    publishers_.init(this);
    services_.init(this);
    
    // Initialize conversion parameters from node parameters
    conversion_params_.frequency_khz = parameters_.frequency_khz;
    conversion_params_.sound_speed_ms = parameters_.sound_speed_ms;
    conversion_params_.range_m = parameters_.range_m;
    conversion_params_.num_beams = parameters_.num_beams;
    conversion_params_.bins_per_beam = parameters_.bins_per_beam;
    conversion_params_.frame_id = parameters_.frame_id;

    // Initialize SDK and set static instance for SDK callback
    instance_ = this;
    initializeGeminiSDK();
    
    // Stop streaming (set offline mode)
    stopPinging();  // Ensure pinging is stopped initially

    RCLCPP_INFO(this->get_logger(), "Gemini Sonar Driver ready. Use services to start/stop sonar.");
}

GeminiSonarNode::~GeminiSonarNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down Gemini Sonar Driver");
    
    stopPinging();
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
    
    if (sonar_streaming_)
    {
        response->success = false;
        response->message = "Sonar is already running";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
    }
    
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

void GeminiSonarNode::handleSvs5Message(unsigned int messageType, unsigned int size, const char* const value)
{
    // Mark that we've received a message from the sonar
    if (!sonar_detected_) {
        sonar_detected_ = true;
        RCLCPP_INFO(this->get_logger(), "Received message from sonar of type %u", messageType);
    }
    last_message_time_ = this->now().nanoseconds();
    
    // Publish raw packet
    auto raw_msg = std::make_shared<gemini_sonar_driver_interfaces::msg::RawPacket>();
    raw_msg->header.stamp = this->now();
    raw_msg->header.frame_id = parameters_.frame_id;
    raw_msg->message_type = messageType;
    raw_msg->data.assign(value, value + size);
    publishers_.raw_packet_->publish(*raw_msg);
    
    // Process based on Svs5 message type
    switch (static_cast<SequencerApi::ESvs5MessageType>(messageType))
    {
        case SequencerApi::GEMINI_STATUS:
        {
            const GLF::GeminiSonarStatusMessage* const statusMsg = reinterpret_cast<const GLF::GeminiSonarStatusMessage*>(value);
            const GLF::GeminiStatusRecord* const pStatus = &statusMsg->m_geminiSonarStatus;
            processGeminiStatus(pStatus);
            break;
        }
            
        case SequencerApi::GLF_LIVE_TARGET_IMAGE:
        {
            GLF::GLogTargetImage* image = (GLF::GLogTargetImage*)value;
            RCLCPP_DEBUG(this->get_logger(), "Received GLF_LIVE_TARGET_IMAGE (sonar data)");
            processGLFImage(*image);
            break;
        }
            
        case SequencerApi::SENSOR_RECORD:
            RCLCPP_DEBUG(this->get_logger(), "Received SENSOR_RECORD");
            // Parse sensor data (GPS, compass, etc.) publish to appropriate topics, might implement later...
            break;
            
        default:
            RCLCPP_DEBUG(this->get_logger(), "Received unhandled Svs5 message type: %u", messageType);
            break;
    }
}

void GeminiSonarNode::processGLFImage(const GLF::GLogTargetImage& image)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    ping_number_++;
    rclcpp::Time timestamp = this->now();
    
    // Extract image data from GLF structure
    const GLF::GMainImage& mainImage = image.m_mainImage;
    
    // Get image dimensions
    const std::vector<UInt8>* imageData = mainImage.m_vecData;
    const std::vector<double>* bearingTable = mainImage.m_vecBearingTable;
    
    // Calculate dimensions
    size_t num_beams = bearingTable->size();
    size_t total_samples = imageData->size();
    size_t samples_per_beam = (num_beams > 0) ? (total_samples / num_beams) : 0;
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "GLF Image: beams=%zu, samples_per_beam=%zu, total=%zu bytes",
                 num_beams, samples_per_beam, total_samples);

    if (ping_number_ <= 5 || ping_number_ % 50 == 0)
    {
        RCLCPP_INFO(this->get_logger(),
            "Ping %u: start_range=%u end_range=%u start_bearing=%u end_bearing=%u compression=%u sos=%.1f freq=%.1f",
            ping_number_,
            mainImage.m_uiStartRange,
            mainImage.m_uiEndRange,
            mainImage.m_uiStartBearing,
            mainImage.m_uiEndBearing,
            mainImage.m_usCompressionType,
            mainImage.m_fSosAtXd,
            mainImage.m_uiModulationFrequency / 1000.0);
    }
    
    // Update conversion parameters from GLF metadata
    conversion_params_.frame_id = parameters_.frame_id;
    conversion_params_.num_beams = num_beams;
    conversion_params_.bins_per_beam = samples_per_beam;
    conversion_params_.frequency_khz = mainImage.m_uiModulationFrequency / 1000.0;
    conversion_params_.sound_speed_ms = mainImage.m_fSosAtXd;
    conversion_params_.start_sample = mainImage.m_uiStartRange;
    
    // Publish msgs using the conversions module
    // auto raw_msg = conversions::createRawSonarImage(beam_data, conversion_params_, timestamp);
    // publishers_.raw_sonar_image_->publish(*raw_msg);
    
    // auto detections_msg = conversions::createSonarDetections(beam_data, conversion_params_, timestamp);
    // publishers_.sonar_detections_->publish(*detections_msg);
    
    // Publish projected image periodically (every 10 pings)
    // if (ping_number_ % 10 == 0)
    // {
    //     auto proj_msg = conversions::createProjectedSonarImage(beam_data, conversion_params_, timestamp);
    //     publishers_.projected_sonar_image_->publish(*proj_msg);
    // }
    
    RCLCPP_DEBUG(this->get_logger(), "Published ping %u", ping_number_);
}

void GeminiSonarNode::processGeminiStatus(const GLF::GeminiStatusRecord* pStatus)
{
    if (!pStatus) return;

    // Format IP address (stored in little-endian format)
    unsigned int ip = pStatus->m_sonarAltIp;
    RCLCPP_DEBUG(this->get_logger(), "Status from %d.%d.%d.%d (device ID: %u)",
        (ip>>0) & 0xFF, (ip>>8) & 0xFF, (ip>>16) & 0xFF, (ip>>24) & 0xFF,
        pStatus->m_deviceID);

    // Check for critical status conditions
    if ((pStatus->m_BOOTSTSRegister & 0x000001ff) == 0x00000001)
    {
        RCLCPP_WARN(this->get_logger(), "Sonar in bootloader mode");
    }
    else if (pStatus->m_shutdownStatus & 0x0001)
    {
        RCLCPP_ERROR(this->get_logger(), "Sonar over temperature!");
    }
    else if (pStatus->m_shutdownStatus & 0x0006)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Sonar out of water");
    }
}

//=============================================================================
// SDK Initialization and Configuration
//=============================================================================

bool GeminiSonarNode::waitForSonarDetection(int timeout_seconds)
{
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(timeout_seconds);
    
    while (!sonar_detected_ && 
           (std::chrono::steady_clock::now() - start_time) < timeout) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return sonar_detected_;
}

bool GeminiSonarNode::initializeGeminiSDK()
{
    RCLCPP_INFO(this->get_logger(), "Initializing Gemini SDK (Svs5Sequencer API)...");
    
    SequencerApi::Svs5Callback callback = [](unsigned int msgType, unsigned int size, const char* const value) {
        instance_->handleSvs5Message(msgType, size, value);
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
    
    // Wait briefly for sonar discovery
    RCLCPP_INFO(this->get_logger(), "Waiting for sonar discovery on network...");
    
    if (waitForSonarDetection(5)) {
        RCLCPP_INFO(this->get_logger(), "Sonar found on network");
    } else {
        RCLCPP_WARN(this->get_logger(), "No sonar detected after 5 second wait");
    }
    
    sdk_initialized_ = true;
    return true;
}

bool GeminiSonarNode::configureSonar()
{   
    RCLCPP_INFO(this->get_logger(), "Configuring Gemini 1200ik sonar via Svs5Sequencer:");
    RCLCPP_INFO(this->get_logger(), "  Range: %.1f m", parameters_.range_m);
    RCLCPP_INFO(this->get_logger(), "  Gain: %.1f %%", parameters_.gain_percent);
    RCLCPP_INFO(this->get_logger(), "  Sound Speed: %d m/s", parameters_.sound_speed_ms);
    RCLCPP_INFO(this->get_logger(), "  Number of Beams: %d", parameters_.num_beams);
    RCLCPP_INFO(this->get_logger(), "  Chirp Mode: %d (0=disabled, 1=enabled, 2=auto)", parameters_.chirp_mode);
    RCLCPP_INFO(this->get_logger(), "  High Resolution: %s (1200ik enhanced range detail)", 
                parameters_.high_resolution ? "ENABLED" : "DISABLED");
    
    Svs5ErrorCode result;
    
    // Configure range
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
    
    // Configure gain
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
    
    // Configure speed of sound (manual mode)
    SequencerApi::SequencerSosConfig sosConfig;
    sosConfig.m_bUsedUserSos = true;  // Use manual SOS TODO: make auto/manual configurable?
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
    
    // Configure image quality (beams)
    SequencerApi::SonarImageQualityLevel qualityLevel;
    if (parameters_.num_beams >= 512) {
        qualityLevel.m_performance = SequencerApi::UL_HIGH_CPU;  // 512-1024 beams
    } else {
        qualityLevel.m_performance = SequencerApi::HIGH_CPU;      // 256-512 beams
    }
    qualityLevel.m_screenPixels = 2048;  // Highest quality TODO: make configurable?
    
    result = SequencerApi::Svs5SetConfiguration(
        SequencerApi::SVS5_CONFIG_CPU_PERFORMANCE,
        sizeof(SequencerApi::SonarImageQualityLevel),
        &qualityLevel,
        parameters_.sonar_id
    );
    if (result != SVS5_SEQUENCER_STATUS_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to set performance level (non-critical)");
    }
    
    // Configure high range resolution for 1200ik
    bool highRes = parameters_.high_resolution;
    result = SequencerApi::Svs5SetConfiguration(
        SequencerApi::SVS5_CONFIG_HIGH_RESOLUTION,
        sizeof(bool),
        &highRes,
        parameters_.sonar_id
    );
    if (result != SVS5_SEQUENCER_STATUS_OK) {
        RCLCPP_WARN(this->get_logger(), "Failed to set high resolution (non-critical)");
    }
    
    // 6. Configure chirp mode
    int chirpMode = parameters_.chirp_mode;
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
    if (!sonar_detected_)
    {
        RCLCPP_WARN(this->get_logger(), "No sonar detected on network - Verify sonar is powered on and connected to network on same subnet i.e 192.168.2.x");
    }

    // Configure sonar settings before starting pinging
    if (!configureSonar()) {
        RCLCPP_ERROR(this->get_logger(), "Sonar configuration failed - cannot start pinging");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Starting sonar pinging...");

    // Configure ping mode from parameters
    SequencerApi::SequencerPingMode pingMode;
    pingMode.m_bFreeRun = parameters_.ping_free_run;
    pingMode.m_msInterval = static_cast<unsigned short>(parameters_.ping_interval_ms);
    pingMode.m_extTTLTrigger = parameters_.ping_ext_trigger;
    
    RCLCPP_INFO(this->get_logger(), "  Ping Mode: %s", 
                parameters_.ping_free_run ? "Free-running (continuous)" : "Interval-based");
    if (!parameters_.ping_free_run) {
        RCLCPP_INFO(this->get_logger(), "  Ping Interval: %d ms (%.1f Hz)", 
                    parameters_.ping_interval_ms, 1000.0 / parameters_.ping_interval_ms);
    }
    if (parameters_.ping_ext_trigger) {
        RCLCPP_INFO(this->get_logger(), "  External TTL Trigger: ENABLED");
    }
    
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
    
    sonar_streaming_ = true;
    RCLCPP_INFO(this->get_logger(), "Sonar streaming started");
    
    if (!waitForSonarDetection(3)) {
        RCLCPP_ERROR(this->get_logger(), "No response from sonar after starting pinging");
        RCLCPP_ERROR(this->get_logger(), "Check that sonar is powered on and on same subnet i.e 192.168.2.x");
        sonar_detected_ = false;
        return false;
    }

    startLogging();
    return true;
}

bool GeminiSonarNode::stopPinging()
{
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
    
    sonar_streaming_ = false;
    stopLogging();
    RCLCPP_INFO(this->get_logger(), "Sonar streaming stopped");
    return true;
}

void GeminiSonarNode::startLogging()
{
    // Get home directory and construct absolute path
    // Note: SDK appends date folders and .glf/.dat filenames automatically, so just provide base directory
    const char* home = std::getenv("HOME");
    if (!home) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get HOME environment variable for logging path");
        return;
    }
    
    std::string log_path = std::string(home) + "/gemini_logs";
    
    Svs5ErrorCode result_log_path = SequencerApi::Svs5SetConfiguration(
        SequencerApi::SVS5_CONFIG_FILE_LOCATION,
        log_path.length() + 1,
        log_path.c_str(),
        parameters_.sonar_id
    );

    if (result_log_path != SVS5_SEQUENCER_STATUS_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set logging path: 0x%08lX", result_log_path);
        return;
    }

    bool start_recording = true;
    Svs5ErrorCode result_record = SequencerApi::Svs5SetConfiguration(
        SequencerApi::SVS5_CONFIG_REC,
        sizeof(bool),
        &start_recording,
        parameters_.sonar_id
    );

    if (result_record != SVS5_SEQUENCER_STATUS_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start Gemini data logging: 0x%08lX", result_record);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Gemini data logging started to directory: %s", log_path.c_str());
        RCLCPP_INFO(this->get_logger(), "  (SDK will create date subdirectories and .glf files automatically)");
    }
}

void GeminiSonarNode::stopLogging()
{
    bool stop_recording = false;
    Svs5ErrorCode result = SequencerApi::Svs5SetConfiguration(
        SequencerApi::SVS5_CONFIG_REC,
        sizeof(bool),
        &stop_recording,
        parameters_.sonar_id
    );
    
    RCLCPP_INFO(this->get_logger(), "Gemini data logging stopped");
}

void GeminiSonarNode::shutdownGeminiSDK()
{
    if (sdk_initialized_)
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down Gemini SDK");
        SequencerApi::StopSvs5();
        sdk_initialized_ = false;
    }
}

NS_FOOT

