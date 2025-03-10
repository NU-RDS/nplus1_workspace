#include "odrive_can.hpp"
#include "kinematics.hpp"
#include "pvPID.hpp"

// Global variables
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;

// Array of ODrives
const int NUM_DRIVES = 3;
const int NUM_JOINTS = 2;
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
float tension_dir[3] = {-1., 1., 1.};
int tensionID = -1;


// init for PID
bool PID = false;
bool got_init = false;
float init_pos[3] = {0.f};
float motor_ang[3] = {0.f};

std::vector<float> target_joint_angles(3);
std::vector<float> current_joint_angles(3);

// PID
using namespace NP1_Kin;

FingerController controller = FingerController(0.1, 0.05, 0.7, 0.1, 0.05, 0.7);

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
    // odrives[index].drive.getCurrent(getCurrents, &odrives[index].user_data);

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
        Serial.print("Command Received: ");
        Serial.println(command);
        
        // Parse command format: "drive_num,direction"
        int commaIndex = command.indexOf(',');
        if (commaIndex != -1) {
            // String cmd = command.substring(commaIndex + 1);
            String cmd = command.substring(commaIndex + 1);
            
            if (cmd == "forward" || cmd == "backward") {
                int jointNum = command.substring(0, commaIndex).toInt();

                if (jointNum >= 0 && jointNum < NUM_JOINTS) {
                    // Your existing rotation logic
                    bool clockwise = cmd.equals("forward");
                    float angle = clockwise ? 5.f : -5.f;
                    target_joint_angles[jointNum] += angle;

                    Serial.println("Moving joint angle.");
                }
            }
            else if (cmd == "ten") {
                int driveNum = command.substring(0, commaIndex).toInt();

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

    Serial.println("All ODrives ready!");
}

void checkErrors(void) {
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
                odrives[driveNum].drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            }
        }
        delay(1);
    }
}

void tension(int &tensionID) {
    if (tensionID != -1)
    {
        Get_Encoder_Estimates_msg_t feedback = odrives[tensionID].user_data.last_feedback;
        float prev_pos = feedback.Pos_Estimate;
        unsigned long last_measurement_time = millis();  
        if (doTension[tensionID] && (!tensioned[tensionID]))
        {
            odrives[tensionID].current_torque = tension_dir[tensionID] * CONSTANT_TORQUE;
            odrives[tensionID].is_running = true;
            odrives[tensionID].drive.setTorque(tension_dir[tensionID] * CONSTANT_TORQUE);
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
                    Serial.print("Tendon ");
                    Serial.print(tensionID);
                    Serial.println(" is tensioned. ");
                    if (tensionID > 0)
                    {
                        odrives[tensionID].current_torque = tension_dir[tensionID] * 0.004f;
                        odrives[tensionID].is_running = true;
                        odrives[tensionID].drive.setTorque(tension_dir[tensionID] * 0.004f);
                    }
                    else
                    {
                        odrives[tensionID].current_torque = tension_dir[tensionID] * 0.015f;
                        odrives[tensionID].is_running = true;
                        odrives[tensionID].drive.setTorque(tension_dir[tensionID] * 0.015f);
                    }
                    tensioned[tensionID] = true;
                    break;
                }
                if (tensionID == 0)
                {
                    odrives[tensionID].current_torque = tension_dir[tensionID] * 0.015f;
                    odrives[tensionID].is_running = true;
                    odrives[tensionID].drive.setTorque(tension_dir[tensionID] * 0.015f);
                }
                else if (tensionID == 1)
                {
                    odrives[tensionID].current_torque = tension_dir[tensionID] * 0.003f;
                    odrives[tensionID].is_running = true;
                    odrives[tensionID].drive.setTorque(tension_dir[tensionID] * 0.003f);
                }
                else if (tensionID == 2)
                {
                    odrives[tensionID].current_torque = tension_dir[tensionID] * 0.004f;
                    odrives[tensionID].is_running = true;
                    odrives[tensionID].drive.setTorque(tension_dir[tensionID] * 0.004f);
                }

                prev_pos = feedback.Pos_Estimate;
                last_measurement_time = current_time;
            }
        }
    }

    // Get initial position (motor_angle) once all tensioned
    if (tensioned[0] && tensioned[1] && tensioned[2] && !got_init)
    {
        pumpEvents(can_intf);
        for (int i = 0; i < NUM_DRIVES; i++)
        {
            Get_Encoder_Estimates_msg_t encoder = odrives[i].user_data.last_feedback;
            init_pos[i] = encoder.Pos_Estimate;
            Serial.print("Motor ");
            Serial.print(i);
            Serial.print(" is at ");
            Serial.println(init_pos[i]);
        }

        std::vector<float> joint_angles = NP1_Kin::motorAngleToJoint(motor_ang[0], motor_ang[1], motor_ang[2]);
        for (int i = 0; i < NUM_JOINTS; i++) {
            current_joint_angles[i] = joint_angles[i];
        }
        Serial.print("Initial joint angles: ");
        Serial.print(current_joint_angles[0]);
        Serial.print(current_joint_angles[1]);
        // set got_init
        got_init = true;
        PID = true;
    }
}

void controlLoop() {
    for (int i = 0; i < NUM_DRIVES; i++)
    {
        Get_Encoder_Estimates_msg_t encoder = odrives[i].user_data.last_feedback;
        // motor ang is motor shaft ang
        motor_ang[i] = NP1_Kin::toShaft((NP1_Kin::RevToDeg(encoder.Pos_Estimate - init_pos[i])));
        Serial.print("Motor ");
        Serial.print(i);
        Serial.print(" is at ");
        Serial.println(motor_ang[i]);
    }
    
    // motor shaft to joint ang
    current_joint_angles = NP1_Kin::motorAngleToJoint(motor_ang[0], motor_ang[1], motor_ang[2]);
    Serial.println("Current joint angles: ");
    Serial.print("Proximal: ");
    Serial.println(current_joint_angles[0]);
    Serial.print("Distal: ");
    Serial.println(current_joint_angles[1]);

    Serial.println("Target joint angles: ");
    Serial.print("Proximal: ");
    Serial.println(target_joint_angles[0]);
    Serial.print("Distal: ");
    Serial.println(target_joint_angles[1]);
    
    std::vector<float> control_torques = controller.computeTorques(target_joint_angles, current_joint_angles);
    Serial.println("Control torques: ");
    Serial.print("Proximal torque: ");
    Serial.println(control_torques[0]);
    Serial.print("Distal torque: ");
    Serial.println(control_torques[1]);

    // Calculate tension forces
    std::vector<float> tendon_force = NP1_Kin::calcTendonForce(control_torques[0], control_torques[1]);
    Serial.println("Tendon forces: ");
    Serial.print("Tendon 1: ");
    Serial.println(tendon_force[1], 5);
    Serial.print("Tendon 2: ");
    Serial.println(tendon_force[2], 5);
    Serial.print("Tendon 3: ");
    Serial.println(tendon_force[0], 5);

    // offset tensions if any require un-tensioning
    std::vector<float> offset_tensions = NP1_Kin::offsetTensions(tendon_force);
    Serial.println("Offset tensions: ");
    Serial.print("Tendon 1: ");
    Serial.println(offset_tensions[1], 5);
    Serial.print("Tendon 2: ");
    Serial.println(offset_tensions[2], 5);
    Serial.print("Tendon 3: ");
    Serial.println(offset_tensions[0], 5);

    std::vector<float> computed_motor_torques = NP1_Kin::tensionToMotorTorque(offset_tensions);
    Serial.println("Raw motor torques: ");
    Serial.print("Tendon torque 1: ");
    Serial.println(computed_motor_torques[1], 5);
    Serial.print("Tendon torque 2: ");
    Serial.println(computed_motor_torques[2], 5);
    Serial.print("Tendon torque 3: ");
    Serial.println(computed_motor_torques[0], 5);

    std::vector<float> scaled_motor_torques = NP1_Kin::scaleTorque(computed_motor_torques);
    Serial.println("Scaled motor torques: ");
    Serial.print("Scaled torque 1: ");
    Serial.println(scaled_motor_torques[1] * tension_dir[1], 16);
    Serial.print("Scaled torque 2: ");
    Serial.println(scaled_motor_torques[2] * tension_dir[2], 16);
    Serial.print("Scaled torque 3: ");
    Serial.println(scaled_motor_torques[0] * tension_dir[0], 16);
    
    // command torque
    for (int i = 0; i < NUM_DRIVES; i++)
    {
        odrives[i].current_torque = 0.5f;
        odrives[i].is_running = true;
        odrives[i].drive.setTorque(scaled_motor_torques[i]*tension_dir[i]);
        Serial.println(scaled_motor_torques[i]*tension_dir[i], 5);
    }
    Serial.println("--------------------------------");
}

void loop() {
    // Serial.println("Pump events.");
    pumpEvents(can_intf);

    // Serial.println("Process serial command");
    processSerialCommand();

    // Serial.println("Tension tendons.");
    // tension(tensionID);
    

    // Serial.println("Enter control loop.");
    if (PID){
        controlLoop();
    }

    // Serial.println("Check for errors.");
    checkErrors();

    delay(1);  // Small delay to prevent overwhelming the system
}