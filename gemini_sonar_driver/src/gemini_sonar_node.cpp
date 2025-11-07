#include "gemini_sonar_node.hpp"
#include <rclcpp/time.hpp>
#include <filesystem>

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
    
    // Logging parameters
    node->declare_parameter("enable_native_logging", enable_native_logging);
    node->declare_parameter("native_log_directory", native_log_directory);
    
    // Topic configuration
    node->declare_parameter("topics.raw_sonar_image", topics.raw_sonar_image);
    node->declare_parameter("topics.projected_sonar_image", topics.projected_sonar_image);
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
    
    node->get_parameter("enable_native_logging", enable_native_logging);
    node->get_parameter("native_log_directory", native_log_directory);
    
    node->get_parameter("topics.raw_sonar_image", topics.raw_sonar_image);
    node->get_parameter("topics.projected_sonar_image", topics.projected_sonar_image);
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
    
    closeNativeLog();
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
    
    // Open native log if requested
    if (request->enable_logging)
    {
        std::string log_dir = request->log_directory.empty() ? 
                             parameters_.native_log_directory : request->log_directory;
        
        if (openNativeLog(log_dir))
        {
            RCLCPP_INFO(this->get_logger(), "Native logging enabled to: %s", log_dir.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to open native log file");
        }
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
        closeNativeLog();
        
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
    // Log to native format if enabled
    if (native_logging_enabled_)
    {
        writeToNativeLog(dataBlock, length, messageType);
    }
    
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
    
    // TODO: Parse ping head structure from Gemini SDK
    // This will extract ping number, timestamp, range, etc.
    
    RCLCPP_DEBUG(this->get_logger(), "Received ping head");
}

void GeminiSonarNode::processPingLine(const char* data, int length)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // TODO: Parse ping line structure from Gemini SDK
    // Each line represents one beam's intensity data
    
    // Store beam data
    std::vector<uint8_t> beam_data(data, data + length);
    current_ping_beams_.push_back(beam_data);
    
    RCLCPP_DEBUG(this->get_logger(), "Received beam %zu", current_ping_beams_.size());
}

void GeminiSonarNode::processPingTail(const char* data, int length)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    ping_complete_ = true;
    ping_number_++;
    
    RCLCPP_DEBUG(this->get_logger(), "Received ping tail - ping complete");
    
    // Publish complete ping as RawSonarImage
    auto raw_msg = createRawSonarImageMsg();
    if (raw_msg)
    {
        publishers_.raw_sonar_image_->publish(*raw_msg);
    }
    
    // Publish SonarDetections (best for FLS data analysis)
    auto detections_msg = createSonarDetectionsMsg();
    if (detections_msg)
    {
        publishers_.sonar_detections_->publish(*detections_msg);
    }
    
    // Publish projected sonar image periodically (every 10 pings)
    if (ping_number_ % 10 == 0)
    {
        auto proj_msg = createProjectedSonarImageMsg();
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
    
    // TODO: Call Gemini SDK configuration functions
    // GEM_SetRange(), GEM_SetGain(), etc.
    // These functions are defined in GeminiCommsPublic.h
    
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
    
    // TODO: Call Gemini SDK function to start pinging
    // GEM_StartPinging() or similar
    
    sonar_running_ = true;
    return true;
}

bool GeminiSonarNode::stopPinging()
{
    RCLCPP_INFO(this->get_logger(), "Stopping sonar pinging...");
    
    // TODO: Call Gemini SDK function to stop pinging
    // GEM_StopPinging() or similar
    
    sonar_running_ = false;
    return true;
}

void GeminiSonarNode::shutdownGeminiSDK()
{
    if (sdk_initialized_)
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down Gemini SDK");
        
        // TODO: Call Gemini SDK shutdown function
        // GEM_ShutdownNetwork() or similar
        
        sdk_initialized_ = false;
    }
}

//=============================================================================
// Native Logging
//=============================================================================

bool GeminiSonarNode::openNativeLog(const std::string& directory)
{
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    // Create directory if it doesn't exist
    std::filesystem::path log_path(directory.empty() ? "." : directory);
    
    if (!std::filesystem::exists(log_path))
    {
        std::filesystem::create_directories(log_path);
    }
    
    // Generate filename with timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << log_path.string() << "/gemini_" 
       << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S")
       << ".gemini";
    
    std::string filename = ss.str();
    
    native_log_file_.open(filename, std::ios::binary | std::ios::out);
    
    if (native_log_file_.is_open())
    {
        native_logging_enabled_ = true;
        RCLCPP_INFO(this->get_logger(), "Opened native log: %s", filename.c_str());
        return true;
    }
    
    RCLCPP_ERROR(this->get_logger(), "Failed to open native log: %s", filename.c_str());
    return false;
}

void GeminiSonarNode::closeNativeLog()
{
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    if (native_log_file_.is_open())
    {
        native_log_file_.close();
        native_logging_enabled_ = false;
        RCLCPP_INFO(this->get_logger(), "Closed native log file");
    }
}

void GeminiSonarNode::writeToNativeLog(const char* data, int length, int messageType)
{
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    if (native_log_file_.is_open())
    {
        // Write message type
        native_log_file_.write(reinterpret_cast<const char*>(&messageType), sizeof(int));
        
        // Write length
        native_log_file_.write(reinterpret_cast<const char*>(&length), sizeof(int));
        
        // Write data
        native_log_file_.write(data, length);
        
        native_log_file_.flush();
    }
}

//=============================================================================
// Message Conversion
//=============================================================================

marine_acoustic_msgs::msg::RawSonarImage::SharedPtr GeminiSonarNode::createRawSonarImageMsg()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!ping_complete_ || current_ping_beams_.empty())
    {
        return nullptr;
    }
    
    auto msg = std::make_shared<marine_acoustic_msgs::msg::RawSonarImage>();
    
    msg->header.stamp = this->now();
    msg->header.frame_id = "gemini";
    
    // Fill ping info
    msg->ping_info.frequency = parameters_.frequency_khz * 1000.0;  // Convert to Hz
    msg->ping_info.sound_speed = parameters_.sound_speed_ms;
    
    msg->sample_rate = 0.0;  // TODO: Get from SDK
    msg->samples_per_beam = parameters_.bins_per_beam;
    msg->sample0 = 0;
    
    // Set beam angles
    msg->tx_angles.resize(current_ping_beams_.size());
    msg->rx_angles.resize(current_ping_beams_.size());
    msg->tx_delays.resize(current_ping_beams_.size(), 0.0f);
    
    for (size_t i = 0; i < current_ping_beams_.size(); ++i)
    {
        double angle_deg = (static_cast<double>(i) - current_ping_beams_.size() / 2.0) * 
                          parameters_.beam_spacing_deg;
        msg->rx_angles[i] = angle_deg * M_PI / 180.0;  // Convert to radians
        msg->tx_angles[i] = 0.0;  // Gemini is receive-only multibeam
    }
    
    // Fill image data
    msg->image.dtype = marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT8;
    msg->image.beam_count = current_ping_beams_.size();
    msg->image.is_bigendian = false;
    
    // Flatten beam data into single vector
    msg->image.data.clear();
    for (const auto& beam : current_ping_beams_)
    {
        msg->image.data.insert(msg->image.data.end(), beam.begin(), beam.end());
    }
    
    return msg;
}

marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr GeminiSonarNode::createProjectedSonarImageMsg()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!ping_complete_ || current_ping_beams_.empty())
    {
        return nullptr;
    }
    
    auto msg = std::make_shared<marine_acoustic_msgs::msg::ProjectedSonarImage>();
    
    msg->header.stamp = this->now();
    msg->header.frame_id = "gemini";
    
    // Fill ping info
    msg->ping_info.frequency = parameters_.frequency_khz * 1000.0;  // Convert to Hz
    msg->ping_info.sound_speed = parameters_.sound_speed_ms;
    
    // Set beam directions as 3D vectors
    msg->beam_directions.resize(current_ping_beams_.size());
    for (size_t i = 0; i < current_ping_beams_.size(); ++i)
    {
        double angle_deg = (static_cast<double>(i) - current_ping_beams_.size() / 2.0) * 
                          parameters_.beam_spacing_deg;
        double angle_rad = angle_deg * M_PI / 180.0;
        
        // Beam direction as unit vector (assuming forward sonar looking in Z direction)
        msg->beam_directions[i].x = 0.0;
        msg->beam_directions[i].y = sin(angle_rad);
        msg->beam_directions[i].z = cos(angle_rad);
    }
    
    // Set range bins
    msg->ranges.resize(parameters_.bins_per_beam);
    for (int i = 0; i < parameters_.bins_per_beam; ++i)
    {
        msg->ranges[i] = (static_cast<float>(i) / parameters_.bins_per_beam) * parameters_.range_m;
    }
    
    // Fill image data
    msg->image.dtype = marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT8;
    msg->image.beam_count = current_ping_beams_.size();
    msg->image.is_bigendian = false;
    
    // Flatten beam data
    msg->image.data.clear();
    for (const auto& beam : current_ping_beams_)
    {
        msg->image.data.insert(msg->image.data.end(), beam.begin(), beam.end());
    }
    
    return msg;
}

marine_acoustic_msgs::msg::SonarDetections::SharedPtr GeminiSonarNode::createSonarDetectionsMsg()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!ping_complete_ || current_ping_beams_.empty())
    {
        return nullptr;
    }
    
    auto msg = std::make_shared<marine_acoustic_msgs::msg::SonarDetections>();
    
    msg->header.stamp = this->now();
    msg->header.frame_id = "gemini";
    
    // Fill ping info
    msg->ping_info.frequency = parameters_.frequency_khz * 1000.0;  // Convert to Hz
    msg->ping_info.sound_speed = parameters_.sound_speed_ms;
    
    size_t num_beams = current_ping_beams_.size();
    
    // Initialize detection flags (all good by default)
    msg->flags.resize(num_beams);
    for (size_t i = 0; i < num_beams; ++i)
    {
        msg->flags[i].flag = marine_acoustic_msgs::msg::DetectionFlag::DETECT_OK;
        // TODO: Add actual beam quality checking from SDK data
    }
    
    // Set two-way travel times for each beam
    msg->two_way_travel_times.resize(num_beams);
    for (size_t i = 0; i < num_beams; ++i)
    {
        // TODO: Get actual detection range from SDK for each beam
        // For now, use maximum range as placeholder
        float detection_range = parameters_.range_m;  // This should be the actual detected range
        msg->two_way_travel_times[i] = (2.0f * detection_range) / parameters_.sound_speed_ms;
    }
    
    // TX delays (zero for single-sector sonar)
    msg->tx_delays.resize(num_beams, 0.0f);
    
    // Intensities (beam amplitudes/strengths)
    msg->intensities.resize(num_beams);
    for (size_t i = 0; i < num_beams; ++i)
    {
        // TODO: Extract actual intensity from beam data
        // For now, use max value in beam as intensity
        if (!current_ping_beams_[i].empty())
        {
            msg->intensities[i] = static_cast<float>(*std::max_element(
                current_ping_beams_[i].begin(), current_ping_beams_[i].end()));
        }
        else
        {
            msg->intensities[i] = 0.0f;
        }
    }
    
    // Set beam angles
    msg->tx_angles.resize(num_beams, 0.0f);  // Along-track (forward/aft) - zero for FLS
    msg->rx_angles.resize(num_beams);
    
    for (size_t i = 0; i < num_beams; ++i)
    {
        // Across-track angles (port/starboard)
        double angle_deg = (static_cast<double>(i) - num_beams / 2.0) * parameters_.beam_spacing_deg;
        msg->rx_angles[i] = static_cast<float>(angle_deg * M_PI / 180.0);
    }
    
    return msg;
}

NS_FOOT
