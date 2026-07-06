/**
 * @file glf_to_rosbag.cpp
 * @brief Offline GLF file processor - converts Gemini log files to ROS2 bag
 * 
 * This utility reads GLF files recorded by the Gemini sonar and converts them
 * to ROS2 bag format using the same optimized glf_processor code as the live driver.
 * 
 * Usage:
 *   ros2 run gemini_sonar_driver glf_to_rosbag <input.glf> <output.bag>
 */

#include "gemini_sonar_driver/package_defs.hpp"
#include "gemini_sonar_driver/glf_processor.hpp"

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

// Gemini SDK GLF Reader API
#include "GenesisSerializer/GlfApi.h"
#include "GenesisSerializer/GlfLoggerGeminiStructure.h"
#include "GenesisSerializer/GeminiStatusRecord.h"

// Gemini Status Messages
#include <gemini_sonar_driver_interfaces/msg/gemini_status.hpp>

// Standard library
#include <sstream>

NS_HEAD

class GLFToROSBag
{
public:
    GLFToROSBag(const std::string& input_glf, const std::string& output_bag, 
                const std::string& frame_id = "gemini")
        : input_glf_(input_glf)
        , output_bag_(output_bag)
        , frame_id_(frame_id)
        , ping_count_(0)
        , status_count_(0)
    {
    }

    bool process()
    {
        RCLCPP_INFO(rclcpp::get_logger("glf_to_rosbag"), 
                    "Processing GLF file: %s", input_glf_.c_str());
        RCLCPP_INFO(rclcpp::get_logger("glf_to_rosbag"), 
                    "Output ROS2 bag: %s", output_bag_.c_str());

        // Open GLF file for reading
        GLF_HANDLE reader = nullptr;
        ErrorCode err = GLF::CreateSingleFileLogReader(&reader, input_glf_.c_str());
        
        if (err != GLF_OK) {
            RCLCPP_ERROR(rclcpp::get_logger("glf_to_rosbag"), 
                        "Failed to open GLF file (error: 0x%08lX)", err);
            return false;
        }

        // Initialize ROS2 bag writer
        if (!initializeBagWriter()) {
            GLF::CloseLogFileHandler(reader);
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("glf_to_rosbag"), "Starting conversion...");

        // Read records sequentially
        while (true) {
            GLF::GlfRecord record;
            err = GLF::GetNextRecord(reader, record);
            
            if (err == GLF_DATA_NOT_AVAILABLE) {
                RCLCPP_INFO(rclcpp::get_logger("glf_to_rosbag"), 
                           "Reached end of GLF file");
                break;
            }
            
            if (err != GLF_OK) {
                RCLCPP_WARN(rclcpp::get_logger("glf_to_rosbag"), 
                           "Error reading record (0x%08lX), stopping", err);
                break;
            }

            // Process the record based on type
            processRecord(record);
        }

        // Cleanup
        GLF::CloseLogFileHandler(reader);
        
        RCLCPP_INFO(rclcpp::get_logger("glf_to_rosbag"), 
                    "Conversion complete! Processed %u sonar pings, %u status msgs", 
                    ping_count_, status_count_);
        
        return true;
    }

private:
    bool initializeBagWriter()
    {
        try {
            rosbag2_storage::StorageOptions storage_options;
            storage_options.uri = output_bag_;
            storage_options.storage_id = "mcap";

            rosbag2_cpp::ConverterOptions converter_options;
            converter_options.input_serialization_format = "cdr";
            converter_options.output_serialization_format = "cdr";

            writer_.open(storage_options, converter_options);

            // Create topic metadata for raw sonar image
            rosbag2_storage::TopicMetadata topic_metadata;
            topic_metadata.name = "/gemini/raw_sonar_image";
            topic_metadata.type = "marine_acoustic_msgs/msg/RawSonarImage";
            topic_metadata.serialization_format = "cdr";
            writer_.create_topic(topic_metadata);

            // Create topic for Gemini status
            topic_metadata.name = "/gemini/status";
            topic_metadata.type = "gemini_sonar_driver_interfaces/msg/GeminiStatus";
            writer_.create_topic(topic_metadata);

            RCLCPP_INFO(rclcpp::get_logger("glf_to_rosbag"), 
                       "Initialized ROS2 bag writer");
            return true;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("glf_to_rosbag"), 
                        "Failed to initialize bag writer: %s", e.what());
            return false;
        }
    }

    void processRecord(const GLF::GlfRecord& record)
    {
        if (!record.m_glfRecord) {
            return;
        }

        // GLF files contain multiple record types - check header to identify
        GLF::GLogTargetImage* targetImage = static_cast<GLF::GLogTargetImage*>(record.m_glfRecord);
        uint8_t dataType = targetImage->m_header.m_ciHeader.m_dataType;
        
        // m_dataType values: 0=SVS5 (sonar), 1=V4, 2=Video, 3=Status, 4=RemV4, etc.
        switch (dataType) {
            case console::DATA_TYPE_SVS5:
                processGeminiImage(*targetImage);
                break;
            
            case console::DATA_TYPE_GEMINI_STATUS:
                processGeminiStatus(record);
                break;
            
            default:
                // Silently skip other record types (V4, video, sensor data, etc.)
                break;
        }
    }

    void processGeminiImage(const GLF::GLogTargetImage& image)
    {
        ping_count_++;
        
        const GLF::GMainImage& mainImage = image.m_mainImage;
        
        // Use the same optimized extraction code as live driver
        glf_processor::PingMetadata metadata = glf_processor::extractPingMetadata(
            mainImage, ping_count_);
        
        // Log progress periodically
        if (ping_count_ % 100 == 0) {
            const bool hf = (metadata.ping_flags & glf_processor::PingFlags::FREQUENCY_MASK) != 0;
            
            // Calculate range using TWTT: range = 0.5 × sound_speed × (samples / sample_rate)
            // modulation_frequency from SDK is already in Hz (not kHz despite the field name)
            const double max_range_m = 0.5 * metadata.sound_speed_ms * 
                                       (metadata.end_range_bin / metadata.modulation_frequency);
            
            RCLCPP_INFO(rclcpp::get_logger("glf_to_rosbag"),
                "Processed %u pings (beams=%u, samples=%u, %s, range=%.1fm)",
                ping_count_, metadata.num_beams, metadata.samples_per_beam,
                hf ? "1200kHz" : "720kHz", max_range_m);
        }

        // Extract beam data
        glf_processor::BeamData beam_data = glf_processor::extractBeamData(
            mainImage, metadata);
        
        if (beam_data.flat_data.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("glf_to_rosbag"),
                       "Empty beam data at ping %u, skipping", ping_count_);
            return;
        }

        // Create ROS message
        auto msg = glf_processor::createRawSonarImage(
            metadata, std::move(beam_data), frame_id_);

        // Write to bag
        writeMessageToBag(msg, metadata.transmit_time_seconds);
    }

    void processGeminiStatus(const GLF::GlfRecord& record)
    {
        status_count_++;
        
        // Cast to Gemini status message structure
        GLF::GeminiSonarStatusMessage* statusMsg = 
            static_cast<GLF::GeminiSonarStatusMessage*>(record.m_glfRecord);
        const GLF::GeminiStatusRecord* pStatus = &statusMsg->m_geminiSonarStatus;
        
        if (!pStatus) {
            RCLCPP_WARN(rclcpp::get_logger("glf_to_rosbag"),
                       "Null status record at count %u, skipping", status_count_);
            return;
        }

        // Format IP address (stored in little-endian format)
        unsigned int ip = pStatus->m_sonarAltIp;
        std::ostringstream ip_stream;
        ip_stream << ((ip >> 0) & 0xFF) << "."
                  << ((ip >> 8) & 0xFF) << "."
                  << ((ip >> 16) & 0xFF) << "."
                  << ((ip >> 24) & 0xFF);
        
        // Create ROS message
        gemini_sonar_driver_interfaces::msg::GeminiStatus status_msg;
        status_msg.header.stamp = rclcpp::Time(0); // GLF doesn't have timestamps for status
        status_msg.header.frame_id = frame_id_;
        status_msg.ip_address = ip_stream.str();
        status_msg.sonar_id = pStatus->m_deviceID;
        status_msg.bootloader_mode = ((pStatus->m_BOOTSTSRegister & 0x000001ff) == 0x00000001);
        status_msg.over_temperature = static_cast<bool>(pStatus->m_shutdownStatus & 0x0001);
        status_msg.out_of_water = static_cast<bool>(pStatus->m_shutdownStatus & 0x0006);
        status_msg.shutdown_status = static_cast<uint16_t>(pStatus->m_shutdownStatus & 0xFFFF);
        status_msg.boot_status = static_cast<uint16_t>(pStatus->m_BOOTSTSRegister & 0xFFFF);

        // Write to bag
        writeStatusToBag(status_msg);
    }

    void writeMessageToBag(
        const marine_acoustic_msgs::msg::RawSonarImage& msg,
        double timestamp_seconds)
    {
        try {
            // Convert timestamp to ROS time
            rclcpp::Time ros_time(static_cast<int64_t>(timestamp_seconds * 1e9));

            // Serialize the message
            rclcpp::Serialization<marine_acoustic_msgs::msg::RawSonarImage> serialization;
            rclcpp::SerializedMessage serialized_msg;
            serialization.serialize_message(&msg, &serialized_msg);

            // Create bag message
            auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            bag_message->topic_name = "/gemini/raw_sonar_image";
            bag_message->recv_timestamp = ros_time.nanoseconds();
            bag_message->send_timestamp = ros_time.nanoseconds();
            bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                new rcutils_uint8_array_t,
                [](rcutils_uint8_array_t* data) {
                    auto fini_return = rcutils_uint8_array_fini(data);
                    delete data;
                    if (fini_return != RCUTILS_RET_OK) {
                        RCLCPP_ERROR(rclcpp::get_logger("glf_to_rosbag"),
                                    "Failed to destroy serialized message");
                    }
                });
            
            *bag_message->serialized_data = serialized_msg.release_rcl_serialized_message();

            writer_.write(bag_message);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("glf_to_rosbag"),
                        "Failed to write message to bag: %s", e.what());
        }
    }

    void writeStatusToBag(const gemini_sonar_driver_interfaces::msg::GeminiStatus& msg)
    {
        try {
            // Use current time since GLF doesn't timestamp status messages
            rclcpp::Time ros_time = rclcpp::Clock().now();

            // Serialize the message
            rclcpp::Serialization<gemini_sonar_driver_interfaces::msg::GeminiStatus> serialization;
            rclcpp::SerializedMessage serialized_msg;
            serialization.serialize_message(&msg, &serialized_msg);

            // Create bag message
            auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            bag_message->topic_name = "/gemini/status";
            bag_message->recv_timestamp = ros_time.nanoseconds();
            bag_message->send_timestamp = ros_time.nanoseconds();
            bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                new rcutils_uint8_array_t,
                [](rcutils_uint8_array_t* data) {
                    auto fini_return = rcutils_uint8_array_fini(data);
                    delete data;
                    if (fini_return != RCUTILS_RET_OK) {
                        RCLCPP_ERROR(rclcpp::get_logger("glf_to_rosbag"),
                                    "Failed to destroy serialized message");
                    }
                });
            
            *bag_message->serialized_data = serialized_msg.release_rcl_serialized_message();

            writer_.write(bag_message);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("glf_to_rosbag"),
                        "Failed to write status to bag: %s", e.what());
        }
    }

    std::string input_glf_;
    std::string output_bag_;
    std::string frame_id_;
    uint32_t ping_count_;
    uint32_t status_count_;
    rosbag2_cpp::Writer writer_;
};

NS_FOOT

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input.glf> <output_bag> [frame_id]" << std::endl;
        std::cerr << "Example: " << argv[0] << " /home/bonnaroo/data/gemini/log.glf ./gemini_data gemini" << std::endl;
        return 1;
    }

    std::string input_glf = argv[1];
    std::string output_bag = argv[2];
    std::string frame_id = (argc >= 4) ? argv[3] : "gemini";

    gemini_sonar_driver::GLFToROSBag converter(input_glf, output_bag, frame_id);
    
    bool success = converter.process();
    
    rclcpp::shutdown();
    
    return success ? 0 : 1;
}
