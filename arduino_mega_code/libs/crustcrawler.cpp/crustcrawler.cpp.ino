#include "crustcrawler.h"
#include "math.h"

CrustCrawler::CrustCrawler() {
  
}

CrustCrawler::~CrustCrawler() {
  _pSerial->end();
}

void CrustCrawler::begin(HardwareSerial &serial, uint8_t controlPin) {
  _ctrlPin = ctrllPin;
  pinMode(_ctrlPin, OUTPUT);

  _pSerial = &serial;
  _pSerial->begin(57600); //To be tested

  CrustCrawler::return_Delay(10);
  CrustCrawler::busWatchdog(0);
  CrustCrawler::returnLevel(PING_READ_INSTRUCTION);
  CrustCrawler::op_Mode(POSITION_MODE);

  for(int i = 0; i < 3; i++) {
    CrustCrawler::min_Pos(_IDs[i], _MIN_ANG[i]);
    CurstCrawler::max_Pos(_IDs[i], _MAX_ANG[i]);
  }
  for(int i = 0; i < 2; i++) {
    CrustCrawler::shadow_id(_GRIP_ID[i], SHA_GRIP_ID);
    CrustCrawler::drive_Mode(_GRIP_ID[i], CCW+i);
    CrustCrawler::max_Pos(_GRIP_ID[i], _MAX_ANG[3]);
    CrustCrawler::min_Pos(_GRIP_ID[i], _MIN_ANG[3]);
  }
}

void CrustCrawler::returnLevel(uint8_t lvl) {
  CrustCrawler::_Instruction <uint8_t> (BROADCAST_ID, STATUS_REUTRN_LEVEL, 0x01, lvl);
}

void CrustCrawler::torque_On(bool Enable) {
  CrustCrawler::_Instruction <uint8_t> (BROADCAST_ID, TORQUE_ENABLE_CHECK, 0x01, Enable);
}

void CrustCrawler::baud(uint8_t baudrate) {
  CrustCrawler::_Instruction <uint8_t> (BROADCAST_ID, BAUD_RATE_CHECK, 0x01, baudrate);
}

void op_Mode(uint8_t mode) {
  CrustCrawler::_Instruction <uint8_t> (BROADCAST_ID, OPERATING_MODE_CHECK, 0x01, mode);
}

void CrustCrawler::shadow_Id(uint8_t act_id, uint8_t sha_id) {
  CrustCrawler::_Instruction <uint8_t> (act_id, SHADOW_ID, 0x01, sha_id);
}

void CrustCrawler::drive_Mode(uint8_t act_id, uint8_t mode) {
  CrustCrawler::_Instruction <uint8_t> (act_id, DRIVE_MODE_CHECK, 0x01, mode);
}

void CrustCrawler::grip_Open() {
  CrustCrawler::_Instruction <uint32_t> (_SHA_GRIP_ID, GOAL_POSITION_CHECK, 0x04, 2450);
}
void CrustCrawler::grip_Close() {
  CrustCrawler::_Iństruction <uint32_t> (_SHA_GRIP_ID, GOAL_POSITION_CHECK, 0x04, 2450);
}

void CrustCrawler::mv_Actuator(uint8_t act_id, int32_t angle, bool dec) {
  if(dec == true) {
    angle = map(angle, -180, 180, 0, 4095);
  }

  CrustCrawler::_Instruction <int32_t> (id, GOAL_POSITION_CHECK, 0x04, angle);
}

void CrustCrawler::mv_Joints(int32_t ang1, int32_t ang2, int32_t ang3, int32_t ang_grip, bool dec){
  if(dec == true) {
    ang1 = map(ang1, -180, 180, 0, 4095);
    ang2 = map(ang2, -180, 180, 0, 4095);
    ang3 = map(ang3, -180, 180, 0, 4095);
    ang_grip = map(ang_grip, -180, 180, 0, 4095);
  }
  CrustCrawler::_syncInstruction <int32_t> (GOAL_POSITION_CHECK, 0x04, ang1, ang2, ang3, ang_grip);  
}

int32_t CrustCrawler::curr_Pos(uint8_t act_id) {
  CrustCrawler::_readInstruction(id, PRESENT_POSITION_CHECK, 0x04);
  CrustCrawler::_statusPacket(0x04);

  if(CrustCrawler::_ErrorFreeCom(); && CrustCrawler::_Checksum(0x04);
  return CrustCrawler::_read <int32_t> (id, 0x04);
}

void CrustCrawler::profile_Vel(uint32_t vel) {
  CrustCrawler::_Instruction <uint32_t> (BROADCAST_ID, PROFILE_VEL, 0x04, vel);
}

void CrustCrawler::profile_Acc(uint32_t acc) {
  CrustCrawler::_Instruction <uint32_t> (BROADCAST_ID, PROFILE_ACC, 0x04, acc);
}

void CrustCrawler::profile_maxAcc(uint32_t lim_acc) {
  CrustCrawler::_Instruction <uint32_t> (BROADCAST_ID, PROFILE_ACC_MAX, 0x04, lim_acc);
}

void CrustCrawler::profile_maxVel(uint32_t lim_vel) {
  CrustCrawler::_Instruction <uint32_t> (BROADCAST_ID, PROFILE_VEL_MAX, 0x04, lim_vel);
}

void CrustCrawler::all_pGain(uint16_t pGain) {
  CrustCrawler::_Instruction <uint16_t> (BROADCAST_ID, POSITION_P_GAIN_CHECK, 0x02, pGain);
}

void CrustCrawler::all_iGain(uint16_t iGain) {
  CrustCrawler::_Instruction <uint16_t> (BROADCAST_ID, POSITION_I_GAIN_CHECK, 0x02, iGain);
}

void CrustCrawler::all_dGain(uint16_t dGain) {
  CrustCrawler::_Instruction <uint16_t> (BROADCAST_ID, POSITION_D_GAIN_CHECK, 0x02, dGain);
}

void CrustCrawler::one_pGain(uint8_t act_id, uint16_t dGain) {
  CrustCrawler::_Instruction <uint16_t> (id, POSITION_P_GAIN_CHECK, 0x02, dGain);
}

void CrustCrawler::one_iGain(uint8_t, act_id, uint16_t iGain) {
  CrustCrawler::_Instruction <uint16_t> (id, POSITION_I_GAIN_CHECK, 0x02, iGain);
}

void CrustCrawler::one_dGain(uint8_t act_id, uint16_t dGain) {
  CrustCrawler::_Instruction <uint16_t> (id, POSITION_D_GAIN_CHECK, 0x02, dGain);
}
