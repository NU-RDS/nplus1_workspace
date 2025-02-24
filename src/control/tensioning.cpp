// #include "tensioning.hpp"


// void addTension(ODriveCAN odrive, bool& hit)
// {
//     Get_Iq_msg_t current = odrive.user_data.last_current.Iq_Measured;
//     if (current > THRES_CURRENT)
//     {
//         hit = true;
//         return;
//     }
//     // TODO: direction
//     odrive.current_torque = CONSTANT_TORQUE;
//     odrive.is_running = true;
//     odrive.drive.setTorque(CONSTANT_TORQUE);
//     delay(10);
// }

// // returns if tensioning is done
// bool autoTension(struct ODriveControl* odrives)
// {
//     for (int i = 0; i < NUM_DRIVES; i++) {
//         // Handle any errors
//         if (odrives[i].user_data.received_heartbeat) {
//             Heartbeat_msg_t heartbeat = odrives[i].user_data.last_heartbeat;
//             if (heartbeat.Axis_Error != 0) {
//                 if (odrives[i].drive.clearErrors()) {
//                     odrives[i].drive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
//                 }
//             }
//         }
//     }

//     bool hit0 = false, hit1 = false, hit2 = false;
//     unsigned long start_time = millis();

//     // Motor 0
//     while (!hit0)
//     {
//         if (millis() - start_time > TIMEOUT_MS) {
//             // Stop all motors on timeout
//             for (int i = 0; i < NUM_DRIVES; i++) {
//                 odrives[i].drive.setTorque(0);
//                 odrives[i].is_running = false;
//             }
//             return false;
//         }
//         ODriveCAN odrive0 = odrives[0];
//         addTension(odrive0, hit0);
//         delay(10);
//     }

//     // Reset timeout for next motor
//     start_time = millis();

//     // Motor 1
//     while (!hit1)
//     {
//         if (millis() - start_time > TIMEOUT_MS) {
//             // Stop all motors on timeout
//             for (int i = 0; i < NUM_DRIVES; i++) {
//                 odrives[i].drive.setTorque(0);
//                 odrives[i].is_running = false;
//             }
//             return false;
//         }
//         ODriveCAN odrive1 = odrives[1];
//         addTension(odrive1, hit1);
//         delay(10);
//     }

//     // Reset timeout for next motor
//     start_time = millis();

//     // Motor 2
//     while (!hit2)
//     {
//         if (millis() - start_time > TIMEOUT_MS) {
//             // Stop all motors on timeout
//             for (int i = 0; i < NUM_DRIVES; i++) {
//                 odrives[i].drive.setTorque(0);
//                 odrives[i].is_running = false;
//             }
//             return false;
//         }
//         ODriveCAN odrive2 = odrives[2];
//         addTension(odrive2, hit2);
//         delay(10);
//     }

//     return hit0 && hit1 && hit2;
// }