#include "odrive_can.hpp"

// Global variables
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;

ODriveCAN odrv0(wrap_can_intf(can_intf), 0);
ODriveUserData odrv0_user_data;

// Constants
const float CONSTANT_TORQUE = 0.0036f;  // Constant torque value
const unsigned long FEEDBACK_DELAY = 100;  // Delay between feedback prints (ms)
const float TARGET_REV = 2.0f;  // Target 5 revolutions

unsigned long start_time = 0;
unsigned long last_feedback_time = 0;
bool motor_running = false;
float initial_pos = 0;
float target_pos = 0;

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
    
    Serial.println("Starting ODrive 5-revolution demo");

    // Register callbacks
    odrv0.onFeedback(onFeedback, &odrv0_user_data);
    odrv0.onStatus(onHeartbeat, &odrv0_user_data);

    // Initialize CAN
    if (!setupCan()) {
        Serial.println("CAN failed to initialize: reset required");
        while (true);
    }

    Serial.println("Found ODrive");
    delay(2000);  // Wait for ODrive to initialize

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
    Serial.println("Press any key to start 5-revolution run...");
    motor_running = false;
}

void loop() {
    pumpEvents(can_intf);
    
    // Handle any errors
    Heartbeat_msg_t heartbeat = odrv0_user_data.last_heartbeat;
    if (heartbeat.Axis_Error != 0) {
        if (odrv0.clearErrors()) {
            odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
        }
        for (int i = 0; i < 15; ++i) {
            delay(1);
            pumpEvents(can_intf);
        }
    }

    // Start motor run when serial input is received
    if (Serial.available() > 0) {
        Serial.read(); // Clear the input buffer
        
        // Store initial position
        pumpEvents(can_intf);
        if (odrv0_user_data.received_feedback) {
            initial_pos = odrv0_user_data.last_feedback.Pos_Estimate;
            target_pos = initial_pos + TARGET_REV;  // Set target to 5 revolutions from current position
            motor_running = true;
            start_time = millis();
            last_feedback_time = 0;
            Serial.println("Starting 5-revolution run...");
            Serial.print("Initial position: ");
            Serial.println(initial_pos);
            Serial.print("Target position: ");
            Serial.println(target_pos);
        }
    }

    if (motor_running) {
        unsigned long current_time = millis();
        
        // Get current position
        if (odrv0_user_data.received_feedback) {
            float current_pos = odrv0_user_data.last_feedback.Pos_Estimate;
            
            // Check if we've reached target position
            if (current_pos < target_pos) {
                // Continue running motor
                odrv0.setTorque(-CONSTANT_TORQUE);
                
                // Print feedback with delay
                if (current_time - last_feedback_time >= FEEDBACK_DELAY) {
                    Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
                    Serial.print("Position (turns): ");
                    Serial.print(feedback.Pos_Estimate);
                    Serial.print(", Target: ");
                    Serial.print(target_pos);
                    Serial.print(", Velocity: ");
                    Serial.println(feedback.Vel_Estimate);
                    last_feedback_time = current_time;
                }
            } else {
                // Target reached, stop motor
                odrv0.setTorque(0.0f);
                motor_running = false;
                Serial.println("5 revolutions completed!");
                Serial.println("Press any key to start another run...");
            }
        }
    }

    delay(1);  // Small delay to prevent overwhelming the system
}