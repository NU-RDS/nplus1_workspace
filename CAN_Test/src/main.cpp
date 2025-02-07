#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "odrive_can_cmd.h" // Include our new header file

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// Example node ID - change this to match your ODrive
const uint8_t NODE_ID = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    
    Serial.println("ODrive CAN Example Starting...");
    
    can1.begin();
    can1.setBaudRate(500000);
    
    // Calculate message ID for encoder estimates
    uint32_t encoderMsgId = ODriveCAN::MessageHandler::getMessageId(
        ODriveCAN::CommandIds::GET_ENCODER_ESTIMATES, 
        NODE_ID
    );
    
    // Set up filter for encoder messages
    can1.setMBFilter(REJECT_ALL);
    can1.setMBFilter(MB0, encoderMsgId);
    
    Serial.print("Listening for encoder messages with ID: 0x");
    Serial.println(encoderMsgId, HEX);
}

void loop() {
    CAN_message_t msg;
    
    if (can1.read(msg)) {
        // Get command type from message ID
        uint32_t cmdId = ODriveCAN::MessageHandler::getCommandId(msg.id);
        
        switch(cmdId) {
            case ODriveCAN::CommandIds::GET_ENCODER_ESTIMATES:
                float position, velocity;
                memcpy(&position, &msg.buf[0], 4);
                memcpy(&velocity, &msg.buf[4], 4);
                
                Serial.print("Position: ");
                Serial.print(position);
                Serial.print(" counts, Velocity: ");
                Serial.print(velocity);
                Serial.println(" counts/s");
                break;
                
            case ODriveCAN::CommandIds::HEARTBEAT:
                Serial.println("Received heartbeat message");
                break;
                
            default:
                Serial.print("Received unknown command ID: 0x");
                Serial.println(cmdId, HEX);
                break;
        }
    }
    
    delay(10);
}