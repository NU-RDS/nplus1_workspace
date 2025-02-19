#include "odrive_can.hpp"

// Global variables
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;

// Array of ODrives
const int NUM_DRIVES = 3;
struct ODriveControl {
    ODriveCAN drive;
    ODriveUserData user_data;
    bool is_running;
    float current_torque;
} odrives[NUM_DRIVES] = {
    {ODriveCAN(wrap_can_intf(can_intf), 0), ODriveUserData(), false, 0.0f},
    {ODriveCAN(wrap_can_intf(can_intf), 1), ODriveUserData(), false, 0.0f},
    {ODriveCAN(wrap_can_intf(can_intf), 2), ODriveUserData(), false, 0.0f}
};

// Constants
const float CONSTANT_TORQUE = 0.0036f;
const unsigned long FEEDBACK_DELAY = 100;  // Delay between feedback prints (ms)
const float TENSION_POS_THRES = 0.01f;

bool tensioned = false;

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
    for (int i = 0; i < NUM_DRIVES; i++) {
        onReceive(msg, odrives[i].drive);
    }
}

bool autoTensioning()
{
    bool tensioned[NUM_DRIVES] = {false, false, false};
    float last_positions[NUM_DRIVES] = {0.0f, 0.0f, 0.0f};
    int current_motor = 0;  // Track which motor we're currently tensioning
    
    // Initial setup for first motor
    last_positions[current_motor] = odrives[current_motor].user_data.last_feedback.Pos_Estimate;
    odrives[current_motor].drive.setTorque(CONSTANT_TORQUE);
    odrives[current_motor].is_running = true;
    odrives[current_motor].current_torque = CONSTANT_TORQUE;
    
    Serial.print("Starting tensioning of Motor ");
    Serial.println(current_motor);
    
    while (!(tensioned[0] && tensioned[1] && tensioned[2])) {
        // Pump CAN events to receive feedback
        pumpEvents(can_intf);
        
        // Only process the current motor
        if (!tensioned[current_motor] && odrives[current_motor].user_data.received_feedback) {
            float current_pos = odrives[current_motor].user_data.last_feedback.Pos_Estimate;
            float position_change = abs(current_pos - last_positions[current_motor]);
            
            // Print progress information
            Serial.print("Motor ");
            Serial.print(current_motor);
            Serial.print(" position change: ");
            Serial.println(position_change, 6);  // Print with 6 decimal places
            
            // Check if position change is below threshold
            if (position_change < TENSION_POS_THRES) {
                tensioned[current_motor] = true;
                odrives[current_motor].drive.setTorque(0); // Stop the motor
                odrives[current_motor].is_running = false;
                odrives[current_motor].current_torque = 0;
                
                Serial.print("Motor ");
                Serial.print(current_motor);
                Serial.println(" tensioned");
                
                // Move to next motor if there is one
                current_motor++;
                if (current_motor < NUM_DRIVES) {
                    Serial.print("Starting tensioning of Motor ");
                    Serial.println(current_motor);
                    
                    // Initialize next motor
                    last_positions[current_motor] = odrives[current_motor].user_data.last_feedback.Pos_Estimate;
                    odrives[current_motor].drive.setTorque(CONSTANT_TORQUE);
                    odrives[current_motor].is_running = true;
                    odrives[current_motor].current_torque = CONSTANT_TORQUE;
                }
            }
            
            last_positions[current_motor] = current_pos;
        }
        
        // Handle any errors during tensioning for the current motor
        if (odrives[current_motor].user_data.received_heartbeat) {
            Heartbeat_msg_t heartbeat = odrives[current_motor].user_data.last_heartbeat;
            if (heartbeat.Axis_Error != 0) {
                Serial.print("Error detected on Motor ");
                Serial.println(current_motor);
                if (odrives[current_motor].drive.clearErrors()) {
                    odrives[current_motor].drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
                }
            }
        }
        
        delay(10); // Small delay to prevent overwhelming the system
    }
    
    Serial.println("All motors tensioned successfully");
    return true;
}

void setupODrive(int index) {
    // Register callbacks
    odrives[index].drive.onFeedback(onFeedback, &odrives[index].user_data);
    odrives[index].drive.onStatus(onHeartbeat, &odrives[index].user_data);

    // Set control mode to torque control
    odrives[index].drive.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, 
                                         ODriveInputMode::INPUT_MODE_PASSTHROUGH);

    // Enable closed loop control
    while (odrives[index].user_data.last_heartbeat.Axis_State != 
           ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrives[index].drive.clearErrors();
        delay(1);
        odrives[index].drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

        for (int i = 0; i < 15; ++i) {
            delay(10);
            pumpEvents(can_intf);
        }
    }
}

void processSerialCommand() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        
        // Parse command format: "drive_num,direction"
        int commaIndex = command.indexOf(',');
        if (commaIndex != -1) {
            int driveNum = command.substring(0, commaIndex).toInt();
            String cmd = command.substring(commaIndex + 1);
            
            if (driveNum >= 0 && driveNum < NUM_DRIVES) {
                if (cmd == "get_current") {
                    // Get current values
                    Get_Iq_msg_t current_msg;
                    if (odrives[driveNum].drive.getCurrents(current_msg)) {
                        Serial.print("ODrive ");
                        Serial.print(driveNum);
                        Serial.print(" Currents - Iq_Setpoint: ");
                        Serial.print(current_msg.Iq_Setpoint);
                        Serial.print(", Iq_Measured: ");
                        Serial.print(current_msg.Iq_Measured);
                    } else {
                        Serial.print("ODrive ");
                        Serial.print(driveNum);
                        Serial.println(" - Failed to get currents");
                    }
                }
                else if (cmd == "true" || cmd == "false") {
                    // Your existing rotation logic
                    bool clockwise = cmd.equals("true");
                    float torque = clockwise ? CONSTANT_TORQUE : -CONSTANT_TORQUE;
                    odrives[driveNum].current_torque = torque;
                    odrives[driveNum].is_running = true;
                    odrives[driveNum].drive.setTorque(torque);

                    Get_Encoder_Estimates_msg_t feedback = odrives[driveNum].user_data.last_feedback;

                    Serial.print("Starting ODrive ");
                    Serial.print(driveNum);
                    Serial.print(clockwise ? " CW " : " CCW" );
                    
                    Serial.print("ODrive ");
                    Serial.print(driveNum);
                    Serial.print(" - Position: ");
                    Serial.print(feedback.Pos_Estimate);
                    Serial.print(", Velocity: ");
                    Serial.println(feedback.Vel_Estimate);
                }
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100);
    
    Serial.println("Starting ODrive GUI Control Demo");

    // Initialize CAN
    if (!setupCan()) {
        Serial.println("CAN failed to initialize: reset required");
        while (true);
    }

    // Initialize all ODrives
    for (int i = 0; i < NUM_DRIVES; i++) {
        Serial.print("Initializing ODrive ");
        Serial.println(i);
        setupODrive(i);
    }

    Serial.println("All ODrives ready!");
}

void loop() {
    pumpEvents(can_intf);

    if (!tensioned)
    {
        tensioned = autoTensioning();
    }
    
    // Process any incoming serial commands
    processSerialCommand();
    
    // Handle all ODrives
    for (int i = 0; i < NUM_DRIVES; i++) {
        // Handle any errors
        if (odrives[i].user_data.received_heartbeat) {
            Heartbeat_msg_t heartbeat = odrives[i].user_data.last_heartbeat;
            if (heartbeat.Axis_Error != 0) {
                if (odrives[i].drive.clearErrors()) {
                    odrives[i].drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
                }
            }
        }
        
        // Print feedback if motor is running
        if (odrives[i].is_running && odrives[i].user_data.received_feedback) {
            static unsigned long last_feedback_time = 0;
            unsigned long current_time = millis();
            
            if (current_time - last_feedback_time >= FEEDBACK_DELAY) {
                // Get_Encoder_Estimates_msg_t feedback = odrives[i].user_data.last_feedback;
                // Serial.print("ODrive ");
                // Serial.print(i);
                // Serial.print(" - Position: ");
                // Serial.print(feedback.Pos_Estimate);
                // Serial.print(", Velocity: ");
                // Serial.println(feedback.Vel_Estimate);
                last_feedback_time = current_time;
            }
        }
    }

    delay(1);  // Small delay to prevent overwhelming the system
}