#include "library.h"
#include <cmath>
#include "arduino.h"
#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"
#include <iostream>

#ifndef DXL_SERIAL
#define DXL_SERIAL Serial

const uint8_t dxl_dir_pin = 2;

CrustCrawler::CrustCrawler() = default;

void CrustCrawler::init_arm(HardwareSerial &dxl_ser, uint8_t _controlPin) {

}

void CrustCrawler::shutdown_arm() {

}

void CrustCrawler::enableTorqueOne(bool enable, uint8_t id) {
    dxl.TorqueOn(0xFE)
}
