#pragma once

#include <Arduino.h>

#include "ODriveCAN.h"
#include "ODriveEnums.h"
#include "ODriveFlexCAN.hpp"

#include <FlexCAN_T4.h>

/*
automated tensioning steps:
1. constant torque to pull on motor zero 
2. until it hits the mechanical limit (when there is a spike in current)
3. pull on motor 1 and 2 then when there is a current spike
*/

const float CONSTANT_TORQUE = 0.0036f; // [Nm]
const float THRES_CURRENT = 8.f; // [A]
const int NUM_DRIVES = 3;
const unsigned long TIMEOUT_MS = 2000; // 2 second timeout

// adds tension on given odrive motor
void addTension(ODriveCAN odrive, bool& hit);

// returns if tensioning is done
bool autoTension(struct ODriveControl* odrives);