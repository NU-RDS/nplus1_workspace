#ifndef ODRIVE_CAN_HPP
#define ODRIVE_CAN_HPP

#include <Arduino.h>

// Include ODrive headers first to prevent naming conflicts
#include "ODriveCAN.h"
#include "ODriveEnums.h"
#include "ODriveFlexCAN.hpp"

// Then include FlexCAN_T4
#undef CAN_ERROR_BUS_OFF  // Undefine the conflicting macro
#include <FlexCAN_T4.h>

// Configuration
#define CAN_BAUDRATE 250000
#define ODRV0_NODE_ID 0

// Forward declaration
struct ODriveStatus;

// User data structure
struct ODriveUserData {
    Heartbeat_msg_t last_heartbeat;
    bool received_heartbeat = false;
    Get_Encoder_Estimates_msg_t last_feedback;
    bool received_feedback = false;
};

// Function declarations
bool setupCan();
void onCanMessage(const CAN_message_t& msg);
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);

// External declarations
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;
extern ODriveCAN odrv0;
extern ODriveUserData odrv0_user_data;
extern ODriveCAN* odrives[];

#endif // ODRIVE_CAN_HPP