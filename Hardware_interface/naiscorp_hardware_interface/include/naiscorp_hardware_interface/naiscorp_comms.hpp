#ifndef NAISCORP_SERIAL_H
#define NAISCORP_SERIAL_H

#include <libserial/SerialPort.h>
#include <string>
#include <vector>
#include <cstdint>

namespace naiscorp
{

    LibSerial::BaudRate convert_baud_rate(int baud_rate);

    class MCUComms
    {
        public:
            MCUComms();

            void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
            void disconnect();
            bool connected() const;

            std::string send_msg(const std::vector<char> &msg_to_send);

            struct MotorState
            {
              uint8_t motor_id;
              float position; // in radians
              float velocity; // in m/s
            };

            std::vector<MotorState> read_state_values();
            void set_motor_values(char motor_id, char command_type, char value);

        private:
            LibSerial::SerialPort serial_conn_;
            int timeout_ms_;
    };

} // namespace naiscorp

#endif // NAISCORP_SERIAL_H