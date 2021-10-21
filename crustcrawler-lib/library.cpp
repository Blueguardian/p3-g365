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

void CrustCrawler::init_arm(HardwareSerial &dxl_ser, HardwareSerial &debug_ser) {
    _pSerial = &dxl_ser;
    _debug_pSerial = &debug_ser;
}

void CrustCrawler::shutdown_arm() {

}

void CrustCrawler::enableTorqueOne(uint8_t id) {
    _pSerial->TorqueOn(id);
}
void CrustCrawler::enableTorqueAll() {
    _pSerial->TorqueOn(BROADCAST_ID);
}
void CrustCrawler::disableTorqueOne(uint8_t id) {
    _pSerial->TorqueOff(id);
}
void CrustCrawler::disableTorqueAll() {
    _pSerial->TorqueOff(BROADCAST_ID);
}
void CrustCrawler::setShadowID(uint8_t id, uint8_t sha_id) {
    _pSerial->writeControlTableItem(0x0C, id, sha_id);
}
void CrustCrawler::setExtremePositions(uint8_t id, uint16_t *expos) {
   uint32_t control_val = _pSerial->readControlTableItem(0x30, id); //Check maximum value
   if(control_val != expos[2]) {
       _pSerial->writeControlTableItem(0x30, id, expos[2]);
   }
   control_val = _pSerial->readControlTableItem(0x34, id);
   if(control_val != expos[1]) {
       _pSerial->writeControlTableItem(0x34, id, expos[1]);
   }
}
void CrustCrawler::grip(bool grip) {
    if(grip == true) {
        CrustCrawler::move_joint(_SHA_ID_GRIP, _ID_GRIP_EXPOS[1]);
    }
    else {
        CrustCrawler::move_joint(_SHA_ID_GRIP, _ID_GRIP_EXPOS[2]);
    }
}

void CrustCrawler::move_joint(uint8_t id, uint16_t theta, char type) {
    if (type == 'R') {
        _pSerial->setGoalPosition(id, theta, UNIT_RAW);
    }
    else if (type == 'D') {
        _pSerial->setGoalPosition(id, theta, UNIT_DEGREE);
    }
    else {
        _debug_pSerial->println("ERROR: Not known type");
    }
}
void
