#include "odrive_can.hpp"
#include "kinematics.hpp"
#include "pvPID.hpp"
#include "tensioning.hpp"

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

PVPID controllers[NUM_DRIVES];

// Constants
const float CONSTANT_TORQUE = 0.0036f;
const unsigned long FEEDBACK_DELAY = 100;  // Delay between feedback prints (ms)

void setupODrive(int index) {
    // Register callbacks
    odrives[index].drive.onFeedback(onFeedback, &odrives[index].user_data);
    odrives[index].drive.onStatus(onHeartbeat, &odrives[index].user_data);
    odrives[index].drive.getCurrent(getCurrents, &odrives[index].user_data);

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
            bool clockwise = command.substring(commaIndex + 1).equals("true");
            
            if (driveNum >= 0 && driveNum < NUM_DRIVES) {
                float torque = clockwise ? CONSTANT_TORQUE : -CONSTANT_TORQUE;
                odrives[driveNum].current_torque = torque;
                odrives[driveNum].is_running = true;
                odrives[driveNum].drive.setTorque(torque);
                
                Serial.print("Starting ODrive ");
                Serial.print(driveNum);
                Serial.println(clockwise ? " CW" : " CCW");
            }
        }
    }
}

void setupController(int index) {
    controllers[index].setPositionGains(1.0, 0.1, 0.01);
    controllers[index].setVelocityGains(0.1, 0.01, 0.001);
    controllers[index].setTorqueLimits(-CONSTANT_TORQUE, CONSTANT_TORQUE);
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
    
    // Auto-tensioning
    bool tensionSuccess = autoTension(odrives);

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
                Get_Encoder_Estimates_msg_t feedback = odrives[i].user_data.last_feedback;
                Serial.print("ODrive ");
                Serial.print(i);
                Serial.print(" - Position: ");
                Serial.print(feedback.Pos_Estimate);
                Serial.print(", Velocity: ");
                Serial.println(feedback.Vel_Estimate);
                last_feedback_time = current_time;
            }

            // PID
            // float torque = controllers[i].update(
            //     odrives[i].user_data.last_feedback.Pos_Estimate,
            //     odrives[i].user_data.last_feedback.Vel_Estimate
            // );
            // odrives[i].drive.setTorque(torque);
        }
    }

    delay(1);  // Small delay to prevent overwhelming the system
}