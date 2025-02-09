#include "odrive_can.hpp"

// Global variables
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;
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

// #include <Arduino.h>
// #include <FlexCAN_T4.h>

// FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;  // Using CAN1 on Teensy

// const int receivePin = 11;  // Define pin 11 for receiving data
// String receivedMessage = ""; // String to store received message
// bool messageComplete = false;

// void canSniff(const CAN_message_t &msg) {
//   // Print message ID
//   Serial.print("CAN MSG ID: ");
//   Serial.print(msg.id, HEX);
  
//   // Print message length
//   Serial.print(" Length: ");
//   Serial.print(msg.len);
  
//   // Print message data bytes
//   Serial.print(" Data: ");
//   for (int i = 0; i < msg.len; i++) {
//     if (msg.buf[i] < 0x10) {
//       Serial.print("0");  // Leading zero for proper formatting
//     }
//     Serial.print(msg.buf[i], HEX);
//     Serial.print(" ");
//   }
  
//   Serial.println();  // New line for next message
// }

// void sendTestMessage() {
//   CAN_message_t msg;
//   msg.id = 0x123;  // Test message ID
//   msg.len = 8;
//   for (int i = 0; i < 8; i++) {
//     msg.buf[i] = i;
//   }
  
//   if (can1.write(msg)) {
//     Serial.println("Test message sent successfully");
//   } else {
//     Serial.println("Failed to send test message");
//   }
// }

// void setup() {
//   Serial.begin(9600);
//   Serial.println("Starting serial...");
//   while (!Serial) {
//     ; // Wait for serial port to connect
//   }
  
//   Serial.println("Serial connected");
  
//   // Configure pin 11 as input for direct reading
//   pinMode(receivePin, INPUT);
//   Serial.println("Pin 11 configured as input");
  
//   // Initialize CAN bus with debug prints
//   Serial.println("Beginning CAN init...");
//   can1.begin();
//   Serial.println("CAN begun");
  
//   Serial.println("Setting baud rate...");
//   can1.setBaudRate(250000);     // Set CAN bus speed to 250kbps
//   Serial.println("Baud rate set");
  
//   Serial.println("Enabling FIFO...");
//   can1.enableFIFO();
//   Serial.println("FIFO enabled");
  
//   Serial.println("Enabling FIFO interrupt...");
//   can1.enableFIFOInterrupt();
//   Serial.println("FIFO interrupt enabled");
  
//   Serial.println("Setting up receive callback...");
//   can1.onReceive(canSniff);
//   Serial.println("Receive callback set");
  
//   Serial.println("Teensy Combined Pin Reader and CAN Bus Ready");
// }

// void processDirectPinRead() {
//   // Read the digital state of pin 11
//   int pinState = digitalRead(receivePin);
  
//   // If we detect a signal
//   if (pinState == HIGH) {
//     receivedMessage += "1";
//   } else {
//     receivedMessage += "0";
//   }
  
//   // Check if we've received a complete message
//   if (receivedMessage.length() >= 8) {
//     // Print the received binary message
//     Serial.print("PIN Received binary message: ");
//     Serial.println(receivedMessage);
    
//     // Convert binary string to ASCII character
//     char asciiChar = 0;
//     for (int i = 0; i < 8; i++) {
//       if (receivedMessage.charAt(i) == '1') {
//         asciiChar |= (1 << (7 - i));
//       }
//     }
    
//     // Print the ASCII character
//     Serial.print("PIN ASCII character: ");
//     Serial.println(asciiChar);
    
//     // Clear the message buffer for the next message
//     receivedMessage = "";
//   }
// }

// void loop() {
//   static unsigned long lastPrint = 0;
//   static unsigned long lastTest = 0;
//   static unsigned long lastPinRead = 0;
  
//   // Process CAN events
//   can1.events();
  
//   // Process direct pin reading every 100ms
//   if (millis() - lastPinRead >= 100) {
//     processDirectPinRead();
//     lastPinRead = millis();
//   }
  
//   // Send a test CAN message every 10 seconds
//   if (millis() - lastTest > 10000) {
//     Serial.println("Attempting to send test message...");
//     sendTestMessage();
//     lastTest = millis();
//   }
  
//   // Print a debug message every 5 seconds
//   if (millis() - lastPrint > 5000) {
//     Serial.println("Still running... waiting for messages");
//     lastPrint = millis();
//   }
  
//   delay(1);  // Small delay to prevent overwhelming the serial output
// }

