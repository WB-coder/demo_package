#include "naiscorp_hardware_interface/naiscorp_comms.hpp"
#include "naiscorp_hardware_interface/constants.h"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>

namespace naiscorp
{
    // Create a static logger for this namespace
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("NaiscorpComms");

    LibSerial::BaudRate convert_baud_rate(int baud_rate)
    {
        // Just handle some common baud rates
        switch (baud_rate)
        {
        case 1200:
            return LibSerial::BaudRate::BAUD_1200;
        case 1800:
            return LibSerial::BaudRate::BAUD_1800;
        case 2400:
            return LibSerial::BaudRate::BAUD_2400;
        case 4800:
            return LibSerial::BaudRate::BAUD_4800;
        case 9600:
            return LibSerial::BaudRate::BAUD_9600;
        case 19200:
            return LibSerial::BaudRate::BAUD_19200;
        case 38400:
            return LibSerial::BaudRate::BAUD_38400;
        case 57600:
            return LibSerial::BaudRate::BAUD_57600;
        case 115200:
            return LibSerial::BaudRate::BAUD_115200;
        case 230400:
            return LibSerial::BaudRate::BAUD_230400;
        default:
            RCLCPP_ERROR(LOGGER, "Error! Baud rate %d not supported! Default to BAUD_115200", baud_rate);
            return LibSerial::BaudRate::BAUD_115200;
        }
    }

    MCUComms::MCUComms() = default;

    void MCUComms::connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    {
        timeout_ms_ = timeout_ms;
        try
        {
            serial_conn_.Open(serial_device);
            serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
        }
        catch (const LibSerial::OpenFailed &e)
        {
            RCLCPP_ERROR(LOGGER, "Failed to open serial port: %s", e.what());
            RCLCPP_ERROR(LOGGER, "Try running with sudo or check if the device exists.");
            throw;
        }
    }

    void MCUComms::disconnect()
    {
        if (serial_conn_.IsOpen())
        {
            serial_conn_.Close();
        }
    }

    bool MCUComms::connected() const
    {
        return serial_conn_.IsOpen();
    }

    std::string MCUComms::send_msg(const std::vector<char> &msg_to_send)
    {
        // Helper function to convert binary data to hex representation
        auto to_hex = [](const std::vector<char> &input)
        {
            std::stringstream ss;
            for (unsigned char c : input)
            {
                ss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                   << static_cast<unsigned int>(static_cast<unsigned char>(c)) << " ";
            }
            return ss.str();
        };

        // Print raw data as hex
        RCLCPP_INFO(LOGGER, "Send Raw Data: %s", to_hex(msg_to_send).c_str());

        serial_conn_.FlushIOBuffers(); // Clear buffers

        // Send raw binary data
        std::string data_str(msg_to_send.begin(), msg_to_send.end());
        serial_conn_.Write(data_str);

        return data_str;
    }

    std::vector<MCUComms::MotorState> MCUComms::read_state_values()
    {
        std::vector<MotorState> motor_states;

        // 1) Gửi lệnh yêu cầu trạng thái
        std::vector<char> req = {
            static_cast<char>(START_BYTE),
            static_cast<char>(PACKET_TYPE_STATE_COMMAND),
            static_cast<char>(PAYLOAD_LENGTH_STATE_COMMAND),
            static_cast<char>(0x00) // payload rỗng → checksum giả định = 0
        };
        send_msg(req);

        // 2) Đọc tất cả byte cho đến khi timeout
        std::vector<unsigned char> buffer;
        unsigned char b;
        while (true)
        {
            try
            {
                serial_conn_.ReadByte(b, timeout_ms_);
                buffer.push_back(b);
            }
            catch (const LibSerial::ReadTimeout &)
            {
                break; // hết data
            }
        }

        // 3) In debug raw dump
        std::stringstream hex_stream;
        hex_stream << "Received " << buffer.size() << " bytes:";
        for (auto byte : buffer)
        {
            hex_stream << " " << std::hex << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
        }
        RCLCPP_INFO(LOGGER, "%s", hex_stream.str().c_str());

        // 4) Tách thành gói 6 byte (START_BYTE + 5 byte data)
        constexpr size_t PACKET_LEN = 10;
        for (size_t idx = 0; idx + PACKET_LEN <= buffer.size(); ++idx)
        {
            if (buffer[idx] != START_BYTE)
            {
                continue;
            }
            // Lấy nguyên gói 6 iiii
            unsigned char motor_id = buffer[idx + 3];
            unsigned char sign = buffer[idx + 4];

            // Đọc 4 byte position và chuyển thành int32_t
            int32_t position = 0;
            position |= (buffer[idx + 5] << 24);
            position |= (buffer[idx + 6] << 16);
            position |= (buffer[idx + 7] << 8);
            position |= buffer[idx + 8];

            int velocity;
            if (sign == 0)
            {
                velocity = buffer[idx + 9];
            }
            else
            {
                velocity = -buffer[idx + 9];
            }
            // buffer[idx+4] có thể là dữ liệu khác hoặc reserved
            // buffer[idx+5] trước đây là checksum, giờ bỏ qua

            // 5) Đổ vào struct
            MotorState st;
            st.motor_id = motor_id;
            st.position = position;
            st.velocity = velocity;
            motor_states.push_back(st);

            RCLCPP_INFO(LOGGER, "Receive: Motor ID=%d POS=%d VEL=%d", static_cast<int>(motor_id), static_cast<int>(position), static_cast<int>(velocity));

            // nhảy qua 6 byte vừa xử lý
            idx += PACKET_LEN - 1;
        }

        return motor_states;
    }

    void MCUComms::set_motor_values(char motor_id, char command_type, char value)
    {
        std::vector<char> packet;

        // Add all bytes to the packet (without checksum initially)
        packet.push_back(START_BYTE);
        packet.push_back(PACKET_TYPE_MOTOR_COMMAND);
        packet.push_back(PAYLOAD_LENGTH_MOTOR_COMMAND);
        packet.push_back(motor_id);
        packet.push_back(command_type);
        packet.push_back(value);

        // Direct calculation of checksum (for verification)
        char direct_checksum = motor_id ^ command_type ^ value;

        // Calculate checksum by XORing all bytes after payload length
        char checksum = 0;
        for (size_t i = 3; i < packet.size(); i++)
        {
            checksum ^= packet[i];
        }

        // Verify checksums match
        if (direct_checksum != checksum)
        {
            RCLCPP_WARN(LOGGER, "Warning: Checksum mismatch. Direct: 0x%02X, Calculated: 0x%02X", 
                       static_cast<int>(direct_checksum & 0xFF), static_cast<int>(checksum & 0xFF));
        }

        // Add the calculated checksum
        packet.push_back(checksum);

        // Pass the vector directly to send_msg, expecting a 7-byte response
        send_msg(packet);
    }


} // namespace naiscorp