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
const float CONSTANT_TORQUE = 0.004f;
const float TENSION_POS_THRES = 0.01f;
const unsigned long FEEDBACK_DELAY = 100;  // Delay between feedback prints (ms)


bool tensioned[3] = {false, false, false};
bool doTension[3] = {false, false, false};
int tensionID = -1;

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

                    Serial.print("Starting ODrive ");
                    Serial.print(driveNum);
                    Serial.println(clockwise ? " CW" : " CCW");
                }
                else if (cmd == "ten") 
                {
                    doTension[driveNum] = true;
                    tensionID = driveNum;
                    if (tensioned[driveNum])
                    {
                        Serial.print("Motor ");
                        Serial.print(tensionID);
                        Serial.println(" is already tensioned");
                    }
                    else
                    {
                        Serial.print("Will tension ");
                        Serial.println(tensionID);
                    }
                }
                //     if (tensioned[driveNum]) {
                //         Serial.print("ODrive ");
                //         Serial.print(driveNum);
                //         Serial.println(" is already tensioned. Ignoring command.");
                //         return;
                //     }
                    
                //     // Initialize position tracking
                //     float prev_pos = odrives[driveNum].user_data.last_feedback.Pos_Estimate;
                //     unsigned long last_movement_time = millis();
                //     const unsigned long STALL_TIMEOUT = 500; // Time in ms to detect a stall
                //     const float MIN_MOVEMENT = 0.001f; // Minimum movement to consider not stalled
                    
                //     Serial.print("Starting tension control for drive ");
                //     Serial.print(driveNum);
                //     Serial.println(" - Applying constant torque until hardstop detected");
                
                //     // Start by applying constant torque
                //     odrives[driveNum].current_torque = CONSTANT_TORQUE;
                //     odrives[driveNum].is_running = true;
                //     odrives[driveNum].drive.setTorque(CONSTANT_TORQUE);
                
                //     while (!tensioned[driveNum]) {
                //         // // Process CAN messages to get new feedback
                //         // pumpEvents(can_intf);
                //                                 // Get current encoder position from feedback
                //         Get_Encoder_Estimates_msg_t feedback = odrives[driveNum].user_data.last_feedback;
                //         float pos_diff = abs(feedback.Pos_Estimate - prev_pos);
                        
                //         Serial.print("Position: ");
                //         Serial.print(feedback.Pos_Estimate);
                //         Serial.print(" Diff: ");
                //         Serial.print(pos_diff);
                        
                //         // Check if we've moved since last check
                //         if (pos_diff > MIN_MOVEMENT) {
                //             // Movement detected, update timestamp
                //             last_movement_time = millis();
                //         } else {
                //             // Check if we've been stalled for longer than timeout
                //             if (millis() - last_movement_time > STALL_TIMEOUT) {
                //                 Serial.println("Hardstop detected! Motor stops.");
                                
                //                 // Maintain holding torque at hardstop
                //                 odrives[driveNum].current_torque = 0.0f;
                //                 odrives[driveNum].is_running = true;
                //                 odrives[driveNum].drive.setTorque(0.0f);
                                
                //                 tensioned[driveNum] = true;
                //                 break;
                //             }
                //         }
                        
                //         // Update previous position for next iteration
                //         prev_pos = feedback.Pos_Estimate;
                        
                //         // Small delay to allow for CAN message processing
                //         delay(10);
                        
                //         // Check for any errors during operation
                //         Heartbeat_msg_t heartbeat = odrives[driveNum].user_data.last_heartbeat;
                //         if (heartbeat.Axis_Error != 0) {
                //             Serial.println("Error detected during tensioning!");
                //             odrives[driveNum].drive.clearErrors();
                //             odrives[driveNum].drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
                //         }
                //     }
                // }
            
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100);
    
    Serial.println("Starting ODrive GUI Tensioning Demo");

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

    // Setting velocity and current limits
    // odrives[0].drive.setLimits(500, 40);
    // odrives[1].drive.setLimits(500, 40);
    // odrives[2].drive.setLimits(500, 40);

    Serial.println("All ODrives ready!");
}

void loop() {
    pumpEvents(can_intf);
    for (int driveNum = 0; driveNum < NUM_DRIVES; driveNum++){
        // Check for errors and print them out
        Heartbeat_msg_t heartbeat = odrives[driveNum].user_data.last_heartbeat;
        if (heartbeat.Axis_Error != 0){
            Get_Error_msg_t msg;
            uint16_t timeout_ms = 50000;
            if (odrives[driveNum].drive.getError(msg, timeout_ms)) {
                Serial.print("Error: ");
                Serial.print(msg.Disarm_Reason);
                Serial.print(" ");
                Serial.println(msg.Active_Errors);
                odrives[driveNum].drive.clearErrors();
                // Serial.println(odrives[driveNum].Axis.motor.alpha_beta_frame_controller.I_bus);
                odrives[driveNum].drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            }
        }
        delay(1);
    }

    processSerialCommand();

    // doTension[0] = false;
    // // Auto tensioning
    // for (int i = 0; i < NUM_DRIVES; i++) 
    // {
    //     Get_Encoder_Estimates_msg_t feedback = odrives[i].user_data.last_feedback;
    //     float prev_pos = feedback.Pos_Estimate;
    //     while (doTension[i] && (!tensioned[i])) // if cmd asks to do tension and if the motor is not yet tensioned
    //     {
    //         pumpEvents(can_intf);
            
            // Get_Encoder_Estimates_msg_t feedback = odrives[i].user_data.last_feedback;
            // float pos_diff = abs(feedback.Pos_Estimate - prev_pos);

            // Serial.print("ODrive ");
            // Serial.print(i);
            // Serial.print(" - Position: ");
            // Serial.print(feedback.Pos_Estimate);
            // Serial.print(" , Prev: ");
            // Serial.print(prev_pos);
            // Serial.print(", Diff: ");
            // Serial.println(pos_diff);

    //         prev_pos = feedback.Pos_Estimate;
    //         delay(10);
    //         Heartbeat_msg_t heartbeat = odrives[i].user_data.last_heartbeat;
    //         if (heartbeat.Axis_Error != 0) {
    //             Serial.println("Error detected during tensioning!");
    //             odrives[i].drive.clearErrors();
    //             odrives[i].drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    //         }
    //     }
    // }
    // // Handle all ODrives
    // int tensionID = 0;
    // doTension[0] = true;
    if (tensionID != -1)
    {
        Get_Encoder_Estimates_msg_t feedback = odrives[tensionID].user_data.last_feedback;
        float prev_pos = feedback.Pos_Estimate;
        unsigned long last_measurement_time = millis();  
        if (doTension[tensionID] && (!tensioned[tensionID]))
        {
            odrives[tensionID].current_torque = CONSTANT_TORQUE;
            odrives[tensionID].is_running = true;
            odrives[tensionID].drive.setTorque(CONSTANT_TORQUE);
        }
        else
        {
            Serial.print("Tendon ");
            Serial.print(tensionID);
            Serial.println(" is tensioned. ");

        }
        while (doTension[tensionID] && (!tensioned[tensionID]))
        {
            pumpEvents(can_intf);
            unsigned long current_time = millis();
            if (current_time - last_measurement_time >= 100)  // 100ms between measurements
            {
                odrives[tensionID].is_running = true;
                Get_Encoder_Estimates_msg_t feedback = odrives[tensionID].user_data.last_feedback;
                float pos_diff = abs(prev_pos - feedback.Pos_Estimate);
                Serial.print("ODrive ");
                Serial.print(tensionID);
                Serial.print(" - Position: ");
                Serial.print(feedback.Pos_Estimate);
                Serial.print(", Diff: ");
                Serial.println(pos_diff);
                Serial.println("------------------------------");

                

                if (pos_diff < TENSION_POS_THRES) 
                {
                    Serial.println("DONE");
                    odrives[tensionID].current_torque = 0.0f;
                    odrives[tensionID].is_running = true;
                    odrives[tensionID].drive.setTorque(0.0f);
                    tensioned[tensionID] = true;
                    break;
                }
                odrives[tensionID].current_torque = CONSTANT_TORQUE;
                odrives[tensionID].is_running = true;
                odrives[tensionID].drive.setTorque(CONSTANT_TORQUE);

                prev_pos = feedback.Pos_Estimate;
                last_measurement_time = current_time;
            }
        }
    }

    // for (int i = 0; i < NUM_DRIVES; i++) {
    //     // Handle any errors
    //     if (odrives[i].user_data.received_heartbeat) {
    //         Heartbeat_msg_t heartbeat = odrives[i].user_data.last_heartbeat;
    //         if (heartbeat.Axis_Error != 0) {
    //             Serial.println("error");
    //             if (odrives[i].drive.clearErrors()) {
    //                 odrives[i].drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    //             }
    //         }
    //     }
        
    //     // Print feedback if motor is running
    //     odrives[tensionID].is_running = true;

    //     if (odrives[tensionID].is_running && odrives[tensionID].user_data.received_feedback) {
    //         unsigned long current_time = millis();
    //         Serial.println("waiting for print");
    //         Get_Encoder_Estimates_msg_t feedback = odrives[i].user_data.last_feedback;
    //         Serial.print("ODrive ");
    //         Serial.print(tensionID);
    //         Serial.print(" - Position: ");
    //         Serial.print(feedback.Pos_Estimate);
    //         Serial.print(", Velocity: ");
    //         Serial.println(feedback.Vel_Estimate);
    //     }
    // }

    delay(1);  // Small delay to prevent overwhelming the system
}