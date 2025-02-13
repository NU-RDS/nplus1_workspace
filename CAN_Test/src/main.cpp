#include "odrive_can.hpp"

// Global variables
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;

ODriveCAN odrv0(wrap_can_intf(can_intf), 0);
ODriveUserData odrv0_user_data;

// Constants
const float CONSTANT_TORQUE = 0.0036f;  // Constant torque value
const unsigned long RUN_DURATION = 3000;  // 3 seconds in milliseconds
const unsigned long FEEDBACK_DELAY = 100;  // Delay between feedback prints (ms)
unsigned long start_time = 0;
unsigned long last_feedback_time = 0;
bool motor_running = false;

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

// Error handling function
void handleErrors() {
    if (odrv0_user_data.received_heartbeat) {
        Heartbeat_msg_t heartbeat = odrv0_user_data.last_heartbeat;
        if (heartbeat.Axis_Error != 0) {
            // Serial.print("Axis Error: 0x");
            // Serial.println(heartbeat.Axis_Error, HEX);
            
            // Check for watchdog timer expiration (0x1000000)
            if (heartbeat.Axis_Error & 0x1000000) {
                Serial.println("Watchdog timer expired, resetting...");
                odrv0.clearErrors();
                delay(100);
                odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            }
        }
    }
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
    onReceive(msg, odrv0);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100);
    
    Serial.println("Starting ODrive 3-second torque run demo");

    // Register callbacks
    odrv0.onFeedback(onFeedback, &odrv0_user_data);
    odrv0.onStatus(onHeartbeat, &odrv0_user_data);

    // Initialize CAN
    if (!setupCan()) {
        Serial.println("CAN failed to initialize: reset required");
        while (true);
    }

    Serial.println("Found ODrive");
    // Reboot the motor
    Serial.println("Rebooting ODrive...");
    // odrv0.reset(ODriveCAN::ResetAction::Reboot);
    delay(2000);  // Wait for ODrive to reboot

    // Set control mode to torque control
    odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);

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

    Serial.println("ODrive ready!");
    Serial.println("Press any key to start 3-second run...");
    motor_running = false;
}

void loop() {
    pumpEvents(can_intf);
    Heartbeat_msg_t heartbeat = odrv0_user_data.last_heartbeat;
    if (heartbeat.Axis_Error != 0)
    {
        // Serial.println(heartbeat.Axis_Error, HEX);
        if (odrv0.clearErrors())
        {
            odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        }
    }
    

    // Start motor run when serial input is received
    if (Serial.available() > 0) {
        Serial.read(); // Clear the input buffer
        
        // TODO: Reset encoder offset and verify
        
        // Add additional verification of encoder reset
        pumpEvents(can_intf);
        if (odrv0_user_data.received_feedback) {
            float initial_pos = odrv0_user_data.last_feedback.Pos_Estimate;
            if (abs(initial_pos) > 0.01f) {  // Check if position is close to zero
                Serial.println("Warning: Encoder reset may not have been successful");
            }
        }
        
        motor_running = true;
        start_time = millis();
        last_feedback_time = 0;
        Serial.println("Starting 3-second run...");
    }

    if (motor_running) {
        unsigned long current_time = millis();
        unsigned long elapsed_time = current_time - start_time;
        odrv0.setTorque(-CONSTANT_TORQUE);

        // Print feedback with delay
        if (current_time - last_feedback_time >= FEEDBACK_DELAY) {
            if (odrv0_user_data.received_feedback) {
                Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
                Serial.print("Position (turns): ");
                Serial.print(feedback.Pos_Estimate);
                Serial.print(", velocity: ");
                Serial.println(feedback.Vel_Estimate);
                last_feedback_time = current_time;
            }
        }

        if (elapsed_time > RUN_DURATION) {
            // Time's up, stop motor
            odrv0.setTorque(0.0f);
            motor_running = false;
            Serial.println("Run complete! Press any key to start another run...");
        }
    }

    delay(10);  // Small delay to prevent overwhelming the system
}