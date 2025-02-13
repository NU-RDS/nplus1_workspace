#include "odrive_can.hpp"

// Global variables
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveUserData odrv0_user_data;
ODriveCAN* odrives[] = {&odrv0};

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
    Serial.println("got msg");
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

    // Initialize CAN
    if (!setupCan()) {
        Serial.println("CAN failed to initialize: reset required");
        while (true);
    }

    Serial.println("Waiting for ODrive...");
    while (!odrv0_user_data.received_heartbeat) {
        pumpEvents(can_intf);
        delay(100);
    }

    Serial.println("found ODrive");

    // Read bus voltage and current
    Get_Bus_Voltage_Current_msg_t vbus;
    if (!odrv0.request(vbus, 1)) {
        Serial.println("vbus request failed!");
        while (true);
    }

    Serial.print("DC voltage [V]: ");
    Serial.println(vbus.Bus_Voltage);
    Serial.print("DC current [A]: ");
    Serial.println(vbus.Bus_Current);

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
}

void loop() {
    pumpEvents(can_intf);

    float SINE_PERIOD = 2.0f;
    float t = 0.001 * millis();
    float phase = t * (TWO_PI / SINE_PERIOD);

    odrv0.setPosition(
        sin(phase),
        cos(phase) * (TWO_PI / SINE_PERIOD)
    );

    if (odrv0_user_data.received_feedback) {
        Get_Encoder_Estimates_msg_t feedback = odrv0_user_data.last_feedback;
        odrv0_user_data.received_feedback = false;
        Serial.print("odrv0-pos:");
        Serial.print(feedback.Pos_Estimate);
        Serial.print(",");
        Serial.print("odrv0-vel:");
        Serial.println(feedback.Vel_Estimate);
    }
}