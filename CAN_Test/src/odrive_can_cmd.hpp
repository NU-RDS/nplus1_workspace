#ifndef ODRIVE_CAN_COMMANDS_HPP
#define ODRIVE_CAN_COMMANDS_HPP

#include <cstdint>  // Use C++ header instead of C header

namespace ODriveCAN {
    // Command IDs
    struct CommandIds {
        // Standard commands
        static const uint32_t HEARTBEAT               = 0x001;
        static const uint32_t ESTOP                   = 0x002;
        static const uint32_t GET_MOTOR_ERROR         = 0x003;
        static const uint32_t GET_ENCODER_ERROR       = 0x004;
        static const uint32_t GET_SENSORLESS_ERROR    = 0x005;
        static const uint32_t SET_AXIS_NODE_ID        = 0x006;
        static const uint32_t SET_AXIS_STATE          = 0x007;
        static const uint32_t GET_ENCODER_ESTIMATES   = 0x009;
        static const uint32_t GET_ENCODER_COUNT       = 0x00A;
        static const uint32_t SET_CONTROLLER_MODES    = 0x00B;
        static const uint32_t SET_INPUT_POS          = 0x00C;
        static const uint32_t SET_INPUT_VEL          = 0x00D;
        static const uint32_t SET_INPUT_TORQUE       = 0x00E;
        static const uint32_t SET_LIMITS             = 0x00F;
        static const uint32_t START_ANTICOGGING      = 0x010;
        static const uint32_t SET_TRAJ_VEL_LIMIT     = 0x011;
        static const uint32_t SET_TRAJ_ACCEL_LIMITS  = 0x012;
        static const uint32_t SET_TRAJ_INERTIA       = 0x013;
        static const uint32_t GET_IQ                 = 0x014;
        static const uint32_t GET_TEMPERATURE        = 0x015;
        static const uint32_t REBOOT                 = 0x016;
        static const uint32_t GET_BUS_VOLTAGE_CURRENT = 0x017;
        static const uint32_t CLEAR_ERRORS           = 0x018;
        static const uint32_t SET_ABSOLUTE_POSITION  = 0x019;
        static const uint32_t SET_POS_GAIN          = 0x01A;
        static const uint32_t SET_VEL_GAINS         = 0x01B;
        static const uint32_t GET_TORQUES           = 0x01C;
    };

    // Helper class to handle CAN message IDs
    class MessageHandler {
    public:
        // Calculate full message ID from command ID and node ID
        static inline uint32_t getMessageId(uint32_t commandId, uint8_t nodeId) {
            return (commandId << 5) | (nodeId & 0x1F);
        }

        // Extract node ID from message ID
        static inline uint8_t getNodeId(uint32_t messageId) {
            return messageId & 0x1F;
        }

        // Extract command ID from message ID
        static inline uint32_t getCommandId(uint32_t messageId) {
            return messageId >> 5;
        }
    };

    // Axis States
    enum class AxisState : uint32_t {
        UNDEFINED                 = 0x00,
        IDLE                     = 0x01,
        STARTUP_SEQUENCE         = 0x02,
        FULL_CALIBRATION_SEQUENCE = 0x03,
        MOTOR_CALIBRATION        = 0x04,
        ENCODER_INDEX_SEARCH     = 0x05,
        ENCODER_OFFSET_CALIBRATION = 0x06,
        CLOSED_LOOP_CONTROL      = 0x07,
        LOCKIN_SPIN             = 0x08,
        ENCODER_DIR_FIND        = 0x09,
        HOMING                  = 0x0A,
        ENCODER_HALL_POLARITY_CALIBRATION = 0x0B,
        ENCODER_HALL_PHASE_CALIBRATION = 0x0C
    };

    // Control Modes
    enum class ControlMode : uint32_t {
        VOLTAGE_CONTROL          = 0x00,
        TORQUE_CONTROL          = 0x01,
        VELOCITY_CONTROL        = 0x02,
        POSITION_CONTROL        = 0x03
    };

    // Input Modes
    enum class InputMode : uint32_t {
        INACTIVE                = 0x00,
        PASSTHROUGH            = 0x01,
        VEL_RAMP               = 0x02,
        POS_FILTER            = 0x03,
        MIX_CHANNELS          = 0x04,
        TRAP_TRAJ             = 0x05,
        TORQUE_RAMP           = 0x06,
        MIRROR                = 0x07,
        TUNING               = 0x08
    };
};

#endif // ODRIVE_CAN_COMMANDS_HPP