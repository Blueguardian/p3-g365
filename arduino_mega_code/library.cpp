/*
 * CrustCrawler library for project on ROB3 group 365 2021
 * Utilizes methods from the Dynamixel2arduino library for methods
 * */


//Include necessary libraries
#include "library.h"
#include <math.h>
#include "Arduino.h"
#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"


//Define values for setGoalPosition units
#define UNIT_RAW 0
#define UNIT_PERCENT 1
#define UNIT_RPM 2
#define UNIT_DEGREE 3
#define UNIT_MILLI_AMPERE 5

using namespace ControlTableItem; //For using controltable item names

CrustCrawler::CrustCrawler() = default; //Constructor for CrustCrawler object

void CrustCrawler::init_arm(DynamixelShield &dxl_ser, HardwareSerial &debug_ser) {
    /**
     * Initialization method for the CrustCrawler arm
     * Sets the shadow ID for the gripper servos and position limits for all joints
     * Enables torque on on servos, closes the gripper and move the CrustCrawler into
     * the starting postion
     * Par: &dxl_ser: Dynamixel serial
     * Par: &debug_ser: Debugging Serial (Used for communication)
     * **/
    _pSerial = &dxl_ser;
    _debug_pSerial = &debug_ser;
    CrustCrawler::setShadowID(4, _SHA_ID_GRIP);
    CrustCrawler::setShadowID(5, _SHA_ID_GRIP);
    CrustCrawler::setExtremePositions(_ID_ARR[3], _ID_GRIP_EXPOS);
    CrustCrawler::setExtremePositions(_ID_ARR[0], _ID_ONE_EXPOS);
    CrustCrawler::setExtremePositions(_ID_ARR[1], _ID_TWO_EXPOS);
    CrustCrawler::setExtremePositions(_ID_ARR[2], _ID_THREE_EXPOS);
    CrustCrawler::enableTorqueAll();
    CrustCrawler::grip(false);
    CrustCrawler::move_joint(2, 2048);
    while (!(CrustCrawler::checkPos(2) < 2053 && CrustCrawler::checkPos(2) > 2038)) {
        _debug_pSerial->print("Motor 2 position: ");
        _debug_pSerial->println(CrustCrawler::checkPos(2));
    }
    CrustCrawler::move_joint(3, 1024);
    while (!(CrustCrawler::checkPos(3) < 1029 && CrustCrawler::checkPos(3) > 1019)) {
        _debug_pSerial->print("Motor 3 position   ");
        _debug_pSerial->println(CrustCrawler::checkPos(3));
    }
    CrustCrawler::move_joint(1, 3072);
    while (!(CrustCrawler::checkPos(1) < 2058 && CrustCrawler::checkPos(1) > 2038)) {
        _debug_pSerial->print("Motor 1 position   ");
        _debug_pSerial->println(CrustCrawler::checkPos(1));
    }

}

void CrustCrawler::shutdown_arm() {
    /**
     * Shutdown method for the CrustCrawler
     * Moves the CrustCrawler into the shutdown position defined here
     * Disables torque on all servos
     * **/
    _debug_pSerial->println("Initializing shutdown");
    CrustCrawler::grip(false);
    CrustCrawler::move_joint(2, 2048);
    while (!(CrustCrawler::checkPos(2) < 2053 && CrustCrawler::checkPos(2) > 2038)) {
        _debug_pSerial->print("Motor 2 position: ");
        _debug_pSerial->println(CrustCrawler::checkPos(2));
    }
    CrustCrawler::move_joint(3, 1024);
    while (!(CrustCrawler::checkPos(3)< 1034 && CrustCrawler::checkPos(3) > 1014)) {
        _debug_pSerial->print("Motor 3 position   ");
        _debug_pSerial->println(CrustCrawler::checkPos(3));
    }
    CrustCrawler::move_joint(1, 3072);
    while (!(CrustCrawler::checkPos(1) < 517 && CrustCrawler::checkPos(1) > 507)) {
        _debug_pSerial->print("Motor 1 position: ");
        _debug_pSerial->println(CrustCrawler::checkPos(1));
    }
    CrustCrawler::move_joint(2, 1174);
    while (!(CrustCrawler::checkPos(2) < 1179 && CrustCrawler::checkPos(2) > 1169)) {
        _debug_pSerial->print("Motor 2 position: ");
        _debug_pSerial->println(CrustCrawler::checkPos(2));
    }
    CrustCrawler::disableTorqueAll();
}

void CrustCrawler::enableTorqueOne(uint8_t id) {
    /**
     * Method for enabling torque on a single servo
     * Using Dynamixel2Arduino library method
     * Par: id: ID of servo
     * **/

    _pSerial->torqueOn(id);
}

void CrustCrawler::enableTorqueAll() {
    /**
     * Method for enabling torque on all servos
     * Using Dynamixel2Arduino library method
     * **/
    _pSerial->torqueOn(BROADCAST_ID);
}

void CrustCrawler::disableTorqueOne(uint8_t id) {
    /**
     * Method for disabling torque on a single servo
     * Using Dynamixel2Arduino library method
     * Par: id: ID of servo
     * **/
    _pSerial->torqueOff(id);
}

void CrustCrawler::disableTorqueAll() {
    /**
     * Method for disabling torque on all servos
     * Using Dynamixel2Arduino library method
     * **/
    _pSerial->torqueOff(BROADCAST_ID);
}

void CrustCrawler::setShadowID(uint8_t id, uint8_t sha_id) {
    /**
     * Method for setting Shadow/Secondary ID for a servo
     * Using Dynamixel2Arduino library method
     * Par: id: ID of servo
     * Par: sha_id: Desired shadown/secondary ID
     * **/
    _pSerial->writeControlTableItem(SECONDARY_ID, id, sha_id);
}

void CrustCrawler::setExtremePositions(uint8_t id, uint16_t *expos) {
    /**
     * Method for setting the maximum and minimum position limits for a servo
     * Using Dynamixel2Arduino library method
     * Par: id: ID of servo
     * Par: *expos: Array of position limits
     * **/
    _pSerial->writeControlTableItem(MAX_POSITION_LIMIT, id, expos[1]);
    _pSerial->writeControlTableItem(MIN_POSITION_LIMIT, id, expos[0]);
}

void CrustCrawler::grip(bool gripper) {
    /**
     * Method for opening and closing the gripper in one move
     * Using Dynamixel2Arduino library method
     * Par: gripper: boolean value true for gripper closed and false for gripper open
     * **/
    if (gripper) {                                                      //If gripper closed
        CrustCrawler::move_joint(_SHA_ID_GRIP, _ID_GRIP_EXPOS[1]);
        _debug_pSerial->println("Gripper not open");                    //Print debugging messages
        _debug_pSerial->print("Motor 4 present position: ");
        _debug_pSerial->println(CrustCrawler::checkPos(4));
        _debug_pSerial->print("Motor 5 present position: ");
        _debug_pSerial->println(CrustCrawler::checkPos(5));
    } else if (!gripper) {                                              //If gripper open
        CrustCrawler::move_joint(_SHA_ID_GRIP, _ID_GRIP_EXPOS[0]);
        _debug_pSerial->println("Gripper not closed");
        _debug_pSerial->print("Motor 4 present position: ");
        _debug_pSerial->println(CrustCrawler::checkPos(4));
        _debug_pSerial->print("Motor 5 present position: ");
        _debug_pSerial->println(CrustCrawler::checkPos(5));
    } else {
        _debug_pSerial->println("ERROR: Wrong parameter");
    }
}

void CrustCrawler::move_joint(uint8_t id, float theta, char type, char controltype) {
    /**
     * Method for moving one servo
     * Depending on the selected type (Input type) and controltype (e.g. Position control) moves
     * the selected joint with the given value (theta) and ID (id)
     * Par: id: ID of servo
     * Par: theta: value to move (e.g. controltype 'P', type 'R', the raw servo angle value)
     * Par: type: Input type (e.g. 'R' for UNIT_RAW)
     * Par: controlType: Control type (e.g. 'P' for position control) NOTE: Does not change controltype of servo
     * **/

    if (controltype == 'P') {
        if (type == 'R') {
            _pSerial->setGoalPosition(id, theta, UNIT_RAW);
        } else if (type == 'D') {
            _pSerial->setGoalPosition(id, theta, UNIT_DEGREE);
        } else {
            _debug_pSerial->println("ERROR: Input unknown type");
        }
    } else if (controltype == 'W') {
        if (type == 'R') {
            _pSerial->setGoalPWM(id, theta, UNIT_RAW);
        } else if (type == 'P') {
            _pSerial->setGoalPWM(id, theta, UNIT_DEGREE);
        } else {
            _debug_pSerial->println("ERROR: Input unknown type");
        }
    } else if (controltype == 'V') {
        if (type == 'R') {
            _pSerial->setGoalVelocity(id, theta, UNIT_RAW);
        } else if (type == 'P') {
            _pSerial->setGoalVelocity(id, theta, UNIT_DEGREE);
        } else if (type == 'M') {
            _pSerial->setGoalVelocity(id, theta, UNIT_RPM);
        } else {
            _debug_pSerial->println("ERROR: Input unknown type");
        }
    } else if (controltype == 'C') {
        if (type == 'R') {
            _pSerial->setGoalCurrent(id, theta, UNIT_RAW);
        } else if (type == 'P') {
            _pSerial->setGoalCurrent(id, theta, UNIT_PERCENT);
        } else if (type == 'A') {
            _pSerial->setGoalCurrent(id, theta, UNIT_MILLI_AMPERE);
        } else {
            _debug_pSerial->println("ERROR: Input unknown type");
        }
    } else {
        _debug_pSerial->println("ERROR: Unknown control type input");
    }
}

void CrustCrawler::move_joints(float theta1, float theta2, float theta3, char type, char controlType) {
    /**
     * Method for simultaneously moving multiple servos
     * Applies the given theta values as movement input depending on the theta value postfix, type and control
     * type as defined in CrustCrawler::move_joint(...)
     * Par: theta1-3: value to move servo with ID 1-3 (further definition in CrustCrawler::move_joint(...))
     * Par: type: Input type as in CrustCrawler::move_joint(...)
     * Par: controltype: Control type as in CrustCrawler::move_joint(...)
     */
    uint16_t stor_arr[3] = {theta1, theta2, theta3};

    for (int i = 0; i < _EXPT_NUM_SERVOS - 2; i++) {
        CrustCrawler::move_joint(i, stor_arr[i], type, controlType);
    }
}

uint32_t CrustCrawler::checkPos(uint8_t id) {
    /**
     * Method for retrieving the current position of a servo
     * Using Dynamixel2Arduino library method
     * Par: id: ID of servo
     * return: pos: position of servo with id
     * **/
    uint32_t pos = _pSerial->getPresentPosition(id);
    return pos;
}

void CrustCrawler::setProfileVel(uint8_t id, uint16_t val) {
    /**
     * Method for setting Profile velocity on control table of the given servo
     * Using Dynamixel2Arduino library method
     * Par: id: ID of servo
     * Par: val: Desired value of Profile velocity
     * **/
    _pSerial->writeControlTableItem(PROFILE_VELOCITY, id, val);
}

void CrustCrawler::setProfileAcc(uint8_t id, uint16_t val) {
    /**
    * Method for setting Profile acceleration on control table of the given servo
    * Using Dynamixel2Arduino library method
    * Par: id: ID of servo
    * Par: val: Desired value of Profile acceleration
    * **/
    _pSerial->writeControlTableItem(PROFILE_ACCELERATION, id, val);
}

void CrustCrawler::setMaxvel(uint8_t id, uint16_t val) {
    /**
    * Method for setting velocity limit on control table of the given servo
    * Using Dynamixel2Arduino library method
    * Par: id: ID of servo
    * Par: val: Desired value of velocity limit
    * **/
    _pSerial->writeControlTableItem(VELOCITY_LIMIT, id, val);
}

void CrustCrawler::setMaxAcc(uint8_t id, uint16_t val) {
    /**
    * Method for setting acceleration limit on control table of the given servo
    * Using Dynamixel2Arduino library method
    * Par: id: ID of servo
    * Par: val: Desired value of acceleration limit
    * **/
    _pSerial->writeControlTableItem(ACCELERATION_LIMIT, id, val);
}

void CrustCrawler::setPGain(uint8_t id, uint16_t val) {
    /**
    * Method for setting P-gain for the PID control in control table for a single servo
    * Using Dynamixel2Arduino library method
    * Par: id: ID of servo
    * Par: val: Desired value of P-gain
    **/
    _pSerial->writeControlTableItem(POSITION_P_GAIN, id, val);
}

void CrustCrawler::setPGainAll(uint16_t val) {
    /**
    * Method for setting P-gain for the PID control in control table for all servos
    * Using Dynamixel2Arduino library method
    * Par: val: Desired value of P-gain
    **/
    for (int i = 0; i < _EXPT_NUM_SERVOS; i++) {
        _pSerial->writeControlTableItem(POSITION_P_GAIN, i, val);
    }
}

void CrustCrawler::setIGain(uint8_t id, uint16_t val) {
    /**
    * Method for setting I-gain for the PID control in control table for a single servo
    * Using Dynamixel2Arduino library method
    * Par: id: ID of servo
    * Par: val: Desired value of I-gain
    **/
    _pSerial->writeControlTableItem(POSITION_I_GAIN, id, val);
}

void CrustCrawler::setIGainAll(uint16_t val) {
    /**
    * Method for setting I-gain for the PID control in control table for all servos
    * Using Dynamixel2Arduino library method
    * Par: val: Desired value of I-gain
    **/
    for (int i = 0; i < _EXPT_NUM_SERVOS; i++) {
        _pSerial->writeControlTableItem(POSITION_I_GAIN, i, val);
    }
}

void CrustCrawler::setDGain(uint8_t id, uint16_t val) {
    /**
    * Method for setting D-gain for the PID control in control table for a single servo
    * Using Dynamixel2Arduino library method
    * Par: id: ID of servo
    * Par: val: Desired value of D-gain
    **/
    _pSerial->writeControlTableItem(POSITION_D_GAIN, id, val);
}

void CrustCrawler::setDGainAll(uint16_t val) {
    /**
    * Method for setting D-gain for the PID control in control table for all servos
    * Using Dynamixel2Arduino library method
    * Par: val: Desired value of D-gain
    **/
    for (int i = 0; i < _EXPT_NUM_SERVOS; i++) {
        _pSerial->writeControlTableItem(POSITION_D_GAIN, i, val);
    }
}

void CrustCrawler::_clearBuffer() {
    /**
     * Method for clearing the buffer
     * Reads all available data in the buffer and discards it
     * **/
    while (_debug_pSerial->available()) {
        _debug_pSerial->read();
    }
}

void CrustCrawler::_statusPacket(uint16_t dataLength) {
    /**
     * Method for handling the statuspacket returned from the DyanmixelServo
     * Unimplemented
     * Par: dataLength: Expected length of the recieved data Packet
     * **/
    //Unknown right now, verify the need for this function
}

/*bool CrustCrawler::_verifyChecksum(uint16_t dataLength) {
   uint16_t checkSum, receivedCheckSum = 0;

    checkSum = CrustCrawler::_update_crc(_arrRx, dataLength + 11);
    receivedCheckSum = _arrRx[dataLength + 10];
    receivedCheckSum = (receivedCheckSum << 8) | _arrRx[dataLength + 9];

    return (checkSum == receivedCheckSum);
}*/

uint16_t CrustCrawler::_update_crc(uint8_t *arr, uint16_t length) {
    /**
     * Method for running a Cyclic Redundancy check on the sent and recieved data
     * Unimplemented
     * Par: *arr: array of data
     * Par: length: Length of data
     * **/
    uint16_t i, j = 0;
    uint16_t crc_accum = 0;

    //Unknown crc_table: taken from Dynamixel2Arduino library
    uint16_t crc_table[256] = {0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E,
                               0x0014, 0x8011, 0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D,
                               0x8027, 0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
                               0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E,
                               0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD,
                               0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE,
                               0x00E4, 0x80E1, 0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE,
                               0x00B4, 0x80B1, 0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D,
                               0x8087, 0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D,
                               0x8197, 0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
                               0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE,
                               0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD,
                               0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E,
                               0x0154, 0x8151, 0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D,
                               0x8167, 0x0162, 0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D,
                               0x8137, 0x0132, 0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E,
                               0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
                               0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E,
                               0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E,
                               0x0374, 0x8371, 0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D,
                               0x8347, 0x0342, 0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE,
                               0x03D4, 0x83D1, 0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED,
                               0x83E7, 0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD,
                               0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
                               0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E,
                               0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD,
                               0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD,
                               0x82F7, 0x02F2, 0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE,
                               0x02C4, 0x82C1, 0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D,
                               0x8257, 0x0252, 0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E,
                               0x0264, 0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
                               0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D,
                               0x8207, 0x0202};

    for (j = 0; j < length; j++) {
        i = ((crc_accum >> 8) ^ arr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}