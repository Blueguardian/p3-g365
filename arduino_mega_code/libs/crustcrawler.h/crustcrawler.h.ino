#include "Arduino.h"

#ifndef CRUSTCRAWLER_C_
#define CRUSTCRAWLER_C_

#define TX_BUFFER_SIZE 40
#define RX_BUFFER_SIZE 40

#define LOW_BYTE(x)  (x & 0xFF)
#define HIGH_BYTE(x) ((x >> 8) & 0xFF)

class CrustCrawler {
private:
    const uint8_t _SERVOS = 5;
    const uint8_t SHA_GRIP_ID = 0x66;
    const uint8_t _IDs[5] = {1, 2, 3, SHA_GRIP_ID};
    const uint8_t _GRIP_ID[2] = {4,5};
    const uint16_t _MIN_ANG[4] = {0, 0, 0, 0};
    const uint16_t _MAX_ANG[4] = {0, 0, 0, 0};
    uint8_t _BUF_TX[TX_BUFFER_SIZE] = {0x00, 0x00, 0x00, 0x00};
    uint8_t _BUF_RX[RX_BUFFER_SIZE] = {0x00, 0x00, 0x00, 0x00};
    uint8_t _ctrlsPin;

public:
    CrustCrawler();
    virtual ~CrustCrawler();

    void begin(HardwareSerial &Serial, uint8_t controlPin);
    void torque_On(bool Enable);
    void returnLevel(uint8_t lvl);
    void return_Delay(uint8_t del);
    void op_Mode(uint8_t mode);
    void baud(uint8_t baudrate);
    void drive_Mode(uint8_t act_id, uint8_t mode);
    void shadow_Id(uint8_t act_id, uint8_t sha_id);
    void busWatchdog(uint8_t val);
    void clr_busWatchdog();

    void max_Pos(uint8_t act_id, uint8_t mxPos);
    void min_Pos(uint8_t act_id, uint8_t mnPos);
    int32_t curr_Pos(uint8_t act_id);

    void mv_Actuator(uint8_t act_id, int32_t angle, bool dec = false);
    void mv_Joints(int32_t ang1, int32_t ang2, int32_t ang3, int32_t ang_grip, bool dec = false);

    void grip_Open();
    void grip_Close();

    void profile_Vel(uint32_t vel);
    void profile_Acc(uint32_t acc);
    void profile_maxAcc(uint32_t lim_acc);
    void profile_maxVel(uint32_t lim_vel);

    void one_PGain(uint8_t act_id, uint16_t dGain);
    void one_IGain(uint8_t act_id, uint16_t iGain);
    void one_DGain(uint8_t act_id, uint16_t dGain);
    void all_pGain(uint16_t pGain);
    void all_iGain(uint16_t iGain);
    void all_dGain(uint16_t dGain);

private:
    bool _ErrorFreeCom();
    bool _Checksum(uint16_t dataLength);
    void _statusPacket(uint16_t dataLength);
    void _readInstruction(uint8_t act_id, uint16_t reg, uint16_t dataLength);
    template<typename ty>ty _read(uint8_t act_id, uint16_t dataLength);
    template<typename ty> void _Instruction(uint8_t act_id, uint16_t reg, uint16_t dataLength, ty value);
    template<typename ty> void _syncInstruction(uint16_t re, uint16_t dataLength, ty data1, ty data2, ty data3, ty data4);
    void _clrBuffer();
    void _Command(uint8_t *a, uint16_t dataLength);
    uint16_t _crc_upd(uint8_t *a, uint16_t dataLength);

    HardwareSerial *_pSerial;
};

#endif
