#include "library.h"
#include <math.h>
#include "Arduino.h"
#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

#define UNIT_RAW 0
#define UNIT_DEGREE 3

const uint8_t dxl_dir_pin = 2;

CrustCrawler::CrustCrawler() = default;

void CrustCrawler::init_arm(DynamixelShield &dxl_ser, HardwareSerial &debug_ser) {
    _pSerial = &dxl_ser;
    _debug_pSerial = &debug_ser;
    _pSerial->torqueOn(2);
    _pSerial->torqueOn(1);
    _pSerial->torqueOn(3);
    this->grip(true);
    _pSerial->setGoalPosition(2, 2048, UNIT_RAW);
    while (!(_pSerial->getPresentPosition(2) < 2053 && _pSerial->getPresentPosition(2) > 2038)) {
        _debug_pSerial->print("Motor 2 position: ");
        _debug_pSerial->println(_pSerial->getPresentPosition(2));
    }
    _pSerial->setGoalPosition(3, 1024, UNIT_RAW);
    while (!(_pSerial->getPresentPosition(3) < 1029 && _pSerial->getPresentPosition(3) > 1019)) {
        _debug_pSerial->print("Motor 3 position   ");
        _debug_pSerial->println(_pSerial->getPresentPosition(3));
    }
    _pSerial->setGoalPosition(1, 1024, UNIT_RAW);
    while (!(_pSerial->getPresentPosition(1) < 1029 && _pSerial->getPresentPosition(1) > 1019)) {
        _debug_pSerial->print("Motor 1 position   ");
        _debug_pSerial->println(_pSerial->getPresentPosition(1));
    }
    this->setExtremePositions(_ID_ARR[1], _ID_ONE_EXPOS);
    this->setExtremePositions(_ID_ARR[2], _ID_TWO_EXPOS);
    this->setExtremePositions(_ID_ARR[3], _ID_THREE_EXPOS);
    this->setExtremePositions(_ID_ARR[4], _ID_GRIP_EXPOS);

}

void CrustCrawler::shutdown_arm() {
    _debug_pSerial->println("Initializing shutdown");
    this->grip(true);
    _pSerial->setGoalPosition(2, 2048, UNIT_RAW);
    while (!(_pSerial->getPresentPosition(2) < 2053 && _pSerial->getPresentPosition(2) > 2038)) {
        _debug_pSerial->print("Motor 2 position: ");
        _debug_pSerial->println(_pSerial->getPresentPosition(2));
    }
    _pSerial->setGoalPosition(3, 1024, UNIT_RAW);
    while (!(_pSerial->getPresentPosition(3) < 1034 && _pSerial->getPresentPosition(3) > 1014)) {
        _debug_pSerial->print("Motor 3 position   ");
        _debug_pSerial->println(_pSerial->getPresentPosition(3));
    }
    _pSerial->setGoalPosition(1, 512, UNIT_RAW);
    while (!(_pSerial->getPresentPosition(1) < 517 && _pSerial->getPresentPosition(1) > 507)) {
        _debug_pSerial->print("Motor 1 position: ");
        _debug_pSerial->println(_pSerial->getPresentPosition(1));
    }
    _pSerial->setGoalPosition(2, 1174, UNIT_RAW);
    while (!(_pSerial->getPresentPosition(2) < 1179 && _pSerial->getPresentPosition(2) > 1169)) {
        _debug_pSerial->print("Motor 2 position: ");
        _debug_pSerial->println(_pSerial->getPresentPosition(1));
    }
    _pSerial->torqueOff(2);
    _pSerial->torqueOff(1);
    _pSerial->torqueOff(3);
}

void CrustCrawler::enableTorqueOne(uint8_t id) {
    _pSerial->torqueOn(id);
}

void CrustCrawler::enableTorqueAll() {
    _pSerial->torqueOn(BROADCAST_ID);
}

void CrustCrawler::disableTorqueOne(uint8_t id) {
    _pSerial->torqueOff(id);
}

void CrustCrawler::disableTorqueAll() {
    _pSerial->torqueOff(BROADCAST_ID);
}

void CrustCrawler::setShadowID(uint8_t id, uint8_t sha_id) {
    _pSerial->writeControlTableItem(0x0C, id, sha_id);
}

void CrustCrawler::setExtremePositions(uint8_t id, uint16_t *expos) {
    uint32_t control_val = _pSerial->readControlTableItem(0x30, id); //Check maximum value
    if (control_val != expos[4]) {
        _pSerial->writeControlTableItem(0x30, id, expos[4]);
    }
    control_val = _pSerial->readControlTableItem(0x34, id);
    if (control_val != expos[3]) {
        _pSerial->writeControlTableItem(0x34, id, expos[3]);
    }
}

void CrustCrawler::grip(bool grip) {
    if (grip) {

        _pSerial->torqueOn(4);
        _pSerial->torqueOn(5);
        this->move_joint(_SHA_ID_GRIP, _ID_GRIP_EXPOS[2]);
        _debug_pSerial->println("Gripper not open");
        _debug_pSerial->print("Motor 4 present position: ");
        _debug_pSerial->println(_pSerial->getPresentPosition(4));
        _debug_pSerial->print("Motor 5 present position: ");
        _debug_pSerial->println(_pSerial->getPresentPosition(5));
        _pSerial->torqueOff(4);
        _pSerial->torqueOff(5);
    } else if (!grip) {
        //  CrustCrawler::
        _pSerial->torqueOn(4);
        _pSerial->torqueOn(5);
        this->move_joint(_SHA_ID_GRIP, _ID_GRIP_EXPOS[1]);
        _debug_pSerial->println("Gripper not closed");
        _debug_pSerial->print("Motor 4 present position: ");
        _debug_pSerial->println(_pSerial->getPresentPosition(4));
        _debug_pSerial->print("Motor 5 present position: ");
        _debug_pSerial->println(_pSerial->getPresentPosition(5));
        _pSerial->torqueOff(4);
        _pSerial->torqueOff(5);
    } else {
        _debug_pSerial->println("ERROR: Wrong parameter");
    }
}

void CrustCrawler::move_joint(uint8_t id, uint16_t theta, char type) {
    if (type == 'R') {
        _pSerial->setGoalPosition(id, theta, UNIT_RAW);
    } else if (type == 'D') {
        _pSerial->setGoalPosition(id, theta, UNIT_DEGREE);
    } else {
        _debug_pSerial->println("ERROR: Input unknown type");
    }
}

void CrustCrawler::move_joints(uint16_t theta1, uint16_t theta2, uint16_t theta3, bool grip_open, char type) {
    uint16_t stor_arr[3] = {theta1, theta2, theta3};
    if (type == 'R') {
        for (int i = 0; i < _EXPT_NUM_SERVOS - 2; i++) {
            CrustCrawler::move_joint(i, stor_arr[i]);
        }
        CrustCrawler::grip(grip_open);
    } else if (type == 'D') {
        for (int i = 0; i < _EXPT_NUM_SERVOS - 2; i++) {
            CrustCrawler::move_joint(i, stor_arr[i], 'D');
        }
        CrustCrawler::grip(grip_open);
    } else {
        _debug_pSerial->println("ERROR: Input unknown type");
    }
}

uint32_t CrustCrawler::checkPos(uint8_t id) {
    _pSerial->getPresentPosition(id);
}

void CrustCrawler::setProfileVel(uint8_t id, uint16_t val) {
    _pSerial->writeControlTableItem(0x70, id, val);
}

void CrustCrawler::setProfileAcc(uint8_t id, uint16_t val) {
    _pSerial->writeControlTableItem(0x6C, id, val);
}

void CrustCrawler::setMaxvel(uint8_t id, uint16_t val) {
    _pSerial->writeControlTableItem(0x2C, id, val);
}

void CrustCrawler::setMaxAcc(uint8_t id, uint16_t val) {
    _pSerial->writeControlTableItem(0x28, id, val);
}

void CrustCrawler::setPGain(uint8_t id, uint16_t val) {
    _pSerial->writeControlTableItem(0x50, id, val);
}

void CrustCrawler::setPGainAll(uint16_t val) {
    for (int i = 0; i < _EXPT_NUM_SERVOS; i++) {
        _pSerial->writeControlTableItem(0x50, i, val);
    }
}

void CrustCrawler::setIGain(uint8_t id, uint16_t val) {
    _pSerial->writeControlTableItem(0x52, id, val);
}

void CrustCrawler::setIGainAll(uint16_t val) {
    for (int i = 0; i < _EXPT_NUM_SERVOS; i++) {
        _pSerial->writeControlTableItem(0x52, i, val);
    }
}

void CrustCrawler::setDGain(uint8_t id, uint16_t val) {
    _pSerial->writeControlTableItem(0x54, id, val);
}

void CrustCrawler::setDGainAll(uint16_t val) {
    for (int i = 0; i < _EXPT_NUM_SERVOS; i++) {
        _pSerial->writeControlTableItem(0x54, i, val);
    }
}

void CrustCrawler::_clearBuffer() {
    while (_debug_pSerial->available()) {
        _debug_pSerial->read();
    }
}

void CrustCrawler::_statusPacket(uint16_t dataLength) {
    //Unknown right now
}

void CrustCrawler::_verifyChecksum(uint16_t dataLength) {
    //Unknown right now
}