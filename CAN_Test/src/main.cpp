#include "odrive_can.hpp"

// Global variables
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;

ODriveCAN odrv0(wrap_can_intf(can_intf), 0);
ODriveUserData odrv0_user_data;

ODriveCAN odrv1(wrap_can_intf(can_intf), 1);
ODriveUserData odrv1_user_data;

ODriveCAN odrv2(wrap_can_intf(can_intf), 2);
ODriveUserData odrv2_user_data;

ODriveCAN* odrives[] = {&odrv0, &odrv1, &odrv2};
ODriveUserData* odriveData[] = {&odrv0_user_data, &odrv1_user_data, &odrv2_user_data};

// Constants for angle conversion
const float DEG_TO_TURNS = 1.0f / 360.0f;  // Convert degrees to turns
const float VELOCITY_FF = 0.25f;  // Velocity feed-forward in turns per second (slower movement)

// CAN setup implementation
bool setupCan() {
    can_intf.begin();
    can_intf.setBaudRate(CAN_BAUDRATE);
    can_intf.setMaxMB(16);
    can_intf.enableFIFO();
    can_intf.enableFIFOInterrupt();
    can_intf.onReceive(onCanMessage);
    return true;
}

// Callback implementations
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
    ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
    odrv_user_data->last_heartbeat = msg;
    odrv_user_data->received_heartbeat = true;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
    ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
    odrv_user_data->last_feedback = msg;
    odrv_user_data->received_feedback = true;
}

void onCanMessage(const CAN_message_t& msg) {
    for (auto odrive : odrives) {
        onReceive(msg, *odrive);
    }
}

void setup() {
    Serial.begin(115200);

    // Wait for serial port
    for (int i = 0; i < 30 && !Serial; ++i) {
        delay(100);
    }
    delay(200);

    Serial.println("Starting ODriveCAN demo");

    // Register callbacks
    odrv0.onFeedback(onFeedback, &odrv0_user_data);
    odrv0.onStatus(onHeartbeat, &odrv0_user_data);
    odrv1.onFeedback(onFeedback, &odrv1_user_data);
    odrv1.onStatus(onHeartbeat, &odrv1_user_data);
    odrv2.onFeedback(onFeedback, &odrv2_user_data);
    odrv2.onStatus(onHeartbeat, &odrv2_user_data);

    // Initialize CAN
    if (!setupCan()) {
        Serial.println("CAN failed to initialize: reset required");
        while (true);
    }

    Serial.println("Waiting for ODrive...");
    while (!odrv0_user_data.received_heartbeat) {
        pumpEvents(can_intf);
        Serial.println("Odrive 0 is ready.");
        delay(100);
    }
    while (!odrv1_user_data.received_heartbeat) {
        pumpEvents(can_intf);
        Serial.println("Odrive 1 is ready.");
        delay(100);
    }
    while (!odrv2_user_data.received_heartbeat) {
        pumpEvents(can_intf);
        Serial.println("Odrive 2 is ready.");
        delay(100);
    }

    Serial.println("Found all ODrives");

    // Set control mode to position control
    odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);

    Serial.println("Enabling closed loop control...");
    while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrv0.clearErrors();
        delay(1);
        odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

        for (int i = 0; i < 15; ++i) {
            delay(10);
            pumpEvents(can_intf);
        }
    }

    Serial.println("ODrive running!");
    Serial.println("Enter angle in degrees to move relative to current position (e.g. 30):");
    Serial.print("Movement speed set to: ");
    Serial.print(VELOCITY_FF);
    Serial.println(" turns per second");
}

void loop() {
    pumpEvents(can_intf);

    // Check if there's serial input available
    if (Serial.available() > 0) {
        // Read the desired angle change in degrees
        float angleChange = Serial.parseFloat();
        Serial.read(); // Clear the newline character
        
        // Get current position
        float currentPos = odrv0_user_data.last_feedback.Pos_Estimate;
        
        // Calculate new position in turns (ODrive uses turns as unit)
        float newPos = currentPos + (angleChange * DEG_TO_TURNS);
        
        // Set new position with velocity feed-forward for smoother movement
        odrv0.setPosition(newPos, VELOCITY_FF, 0.0f);
        
        Serial.print("Moving to new position: ");
        Serial.print(newPos);
        Serial.print(" turns at ");
        Serial.print(VELOCITY_FF);
        Serial.println(" turns/second");
    }

    // Print feedback if available
    if (odrv0_user_data.received_feedback) {
        Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
        odrv0_user_data.received_feedback = false;
        Serial.print("Current position (turns): ");
        Serial.print(feedback.Pos_Estimate);
        Serial.print(", velocity: ");
        Serial.println(feedback.Vel_Estimate);
    }
}