#include "odrive_can.hpp"

// Global variables
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;

// Array of ODrives
ODriveCAN odrv[3] = {
    ODriveCAN(wrap_can_intf(can_intf), 0),
    ODriveCAN(wrap_can_intf(can_intf), 1),
    ODriveCAN(wrap_can_intf(can_intf), 2)
};
ODriveUserData odrv_user_data[3];

// Constants
const float CONSTANT_TORQUE = 0.0036f;  // Constant torque value
const unsigned long FEEDBACK_DELAY = 100;  // Delay between feedback prints (ms)
const float TARGET_REV = 1.0f;  // Target 1 revolution per step

// Global state variables
unsigned long start_time = 0;
unsigned long last_feedback_time = 0;
bool motor_running = false;
float initial_pos = 0;
float target_pos = 0;
int selected_odrive = 0;
bool rotation_direction = true;

// User input variables
int odrive_index = 0;
bool direction = true;

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
void handleErrors(int index) {
    if (odrv_user_data[index].received_heartbeat) {
        Heartbeat_msg_t heartbeat = odrv_user_data[index].last_heartbeat;
        if (heartbeat.Axis_Error != 0) {
            if (heartbeat.Axis_Error & 0x1000000) {
                Serial.println("Watchdog timer expired, resetting...");
                odrv[index].clearErrors();
                delay(100);
                odrv[index].setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            }
        }
    }
}

// Motor step function
void step() {
    if (!motor_running) {
        // Store initial position
        pumpEvents(can_intf);
        if (odrv_user_data[selected_odrive].received_feedback) {
            initial_pos = odrv_user_data[selected_odrive].last_feedback.Pos_Estimate;
            target_pos = initial_pos + (rotation_direction ? TARGET_REV : -TARGET_REV);
            motor_running = true;
            start_time = millis();
            last_feedback_time = 0;
            Serial.print("Starting 1 revolution for ODrive ");
            Serial.print(selected_odrive);
            Serial.println(rotation_direction ? " forward..." : " backward...");
            Serial.print("Initial position: ");
            Serial.println(initial_pos);
            Serial.print("Target position: ");
            Serial.println(target_pos);
        }
    }

    if (motor_running) {
        unsigned long current_time = millis();
        
        // Get current position
        if (odrv_user_data[selected_odrive].received_feedback) {
            float current_pos = odrv_user_data[selected_odrive].last_feedback.Pos_Estimate;
            
            // Check if we've reached target position
            bool target_reached = rotation_direction ? 
                (current_pos >= target_pos) : (current_pos <= target_pos);
            
            if (!target_reached) {
                // Continue running motor
                float torque = rotation_direction ? CONSTANT_TORQUE : -CONSTANT_TORQUE;
                odrv[selected_odrive].setTorque(torque);
                
                // Print feedback with delay
                if (current_time - last_feedback_time >= FEEDBACK_DELAY) {
                    Get_Encoder_Estimates_msg_t feedback = odrv_user_data[selected_odrive].last_feedback;
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
                odrv[selected_odrive].setTorque(0.0f);
                motor_running = false;
                Serial.println("1 revolution completed!");
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
    for (int i = 0; i < 3; i++) {
        onReceive(msg, odrv[i]);
    }
}

// Get user input for ODrive selection and direction
void getUserInput() {
    Serial.println("Enter ODrive index (0-2):");
    while (!Serial.available()) {
        delay(100);
    }
    odrive_index = Serial.parseInt();
    Serial.read(); // Clear newline
    Serial.print("Selected ODrive: ");
    Serial.println(odrive_index);

    Serial.println("Enter direction (1 for forward, 0 for backward):");
    while (!Serial.available()) {
        delay(100);
    }
    direction = Serial.parseInt() != 0;
    Serial.read(); // Clear newline
    Serial.print("Selected direction: ");
    Serial.println(direction ? "forward" : "backward");
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100);
    
    Serial.println("Starting ODrive control demo");

    // Register callbacks for all ODrives
    for (int i = 0; i < 3; i++) {
        odrv[i].onFeedback(onFeedback, &odrv_user_data[i]);
        odrv[i].onStatus(onHeartbeat, &odrv_user_data[i]);
    }

    // Initialize CAN
    if (!setupCan()) {
        Serial.println("CAN failed to initialize: reset required");
        while (true);
    }

    Serial.println("Found ODrives");
    delay(2000);  // Wait for ODrives to initialize

    // Get initial user input
    getUserInput();
    selected_odrive = odrive_index;
    rotation_direction = direction;

    // Set control mode for selected ODrive
    odrv[selected_odrive].setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);

    Serial.println("Enabling closed loop control...");
    while (odrv_user_data[selected_odrive].last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrv[selected_odrive].clearErrors();
        delay(1);
        odrv[selected_odrive].setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

        for (int i = 0; i < 15; ++i) {
            delay(10);
            pumpEvents(can_intf);
        }
    }

    Serial.println("ODrive ready!");
}

void loop() {
    pumpEvents(can_intf);
    
    // Handle any errors for selected ODrive
    handleErrors(selected_odrive);

    // Check for new user input
    if (Serial.available() > 0) {
        getUserInput();
        selected_odrive = odrive_index;
        rotation_direction = direction;
        
        // Set control mode for newly selected ODrive
        odrv[selected_odrive].setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
        odrv[selected_odrive].setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    }

    // Execute one step
    step();

    delay(1);  // Small delay to prevent overwhelming the system
}