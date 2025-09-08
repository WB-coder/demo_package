#ifndef NAISCORP_CONSTANTS_H
#define NAISCORP_CONSTANTS_H

namespace naiscorp
{

    // General packet structure constants
    constexpr unsigned char START_BYTE = 0xAA;

    // Packet types
    enum PacketType
    {
        PACKET_TYPE_MOTOR_COMMAND = 0x01, /* ROS2 → MCU */
        PACKET_TYPE_MOTOR_STATE = 0x02,   /* MCU → ROS2 */
        PACKET_TYPE_PID_COMMAND = 0x03,   /* ROS2 → MCU */
        PACKET_TYPE_STATE_COMMAND = 0x04  /* ROS2 → MCU */
    };

    // Command types
    enum CommandType
    {
        COMMAND_TYPE_VELOCITY = 0x01, /* For motors 1-2 */
        // Position commands are not used for differential drive robot
        // COMMAND_TYPE_POSITION = 0x02,  /* For motors 3-7 */
        COMMAND_TYPE_VELOCITY_MINUS = 0x03, /* For motors 1-2 */
    };

    // Motor IDs
    enum MotorID
    {
        LEFT_WHEEL = 0x01,
        RIGHT_WHEEL = 0x02,
        // The following motor IDs are not used for the differential drive robot:
        /*
        LEFT_LEG_JOINT = 0x03,
        RIGHT_LEG_JOINT = 0x04,
        LEFT_ARM_JOINT = 0x05,
        RIGHT_ARM_JOINT = 0x06,
        HEAD_JOINT = 0x07
        */
    };

    // Payload length
    enum PayloadLength
    {
        PAYLOAD_LENGTH_MOTOR_COMMAND = 3, /* ROS2 → MCU */
        PAYLOAD_LENGTH_MOTOR_STATE = 3,   /* MCU → ROS2 */
        PAYLOAD_LENGTH_PID_COMMAND = 4,   /* ROS2 → MCU */
        PAYLOAD_LENGTH_STATE_COMMAND = 0  /* ROS2 → MCU */
    };

} // namespace naiscorp

#endif // NAISCORP_CONSTANTS_H