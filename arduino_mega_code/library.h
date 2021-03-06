#include <math.h>
#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <DynamixelShield.h>

#ifndef CRUSTCRAWLER_H
#define CRUSTCRAWLER_H

#define BUF_SIZE 40 //Buffer size
#define BROADCAST_ID 0xFE //Broadcast ID for all motors
#define _OFFSET_MARGIN 10 //Allowed deviance

class CrustCrawler {
private:
    const uint8_t _EXPT_NUM_SERVOS = 5; //Expected number of servos available
    uint8_t _SHA_ID_GRIP = 0x10; //Gripper shadow ID for motor four and five
    uint8_t _ID_ARR[4] = { 1, 2, 3, _SHA_ID_GRIP}; //Array of motor IDs
    uint16_t _ID_ONE_EXPOS[4] = {512, 3584};  //Extremum positions of motor one
    uint16_t _ID_TWO_EXPOS[4] = {800, 2330}; //Extremum positions of motor two
    uint16_t _ID_THREE_EXPOS[4] = {800, 2547};  //Extremum positions of motor three
    uint16_t _ID_GRIP_EXPOS[4] = {1100, 2000};  //Extremum positions of motor four (gripper 1)
    DynamixelShield *_pSerial; //Pointer to Dynamixel serial
    HardwareSerial *_debug_pSerial; //Pointer to debug serial

public:
    CrustCrawler(); //Standard constructor
    void init_arm(DynamixelShield &dxl_ser, HardwareSerial &debug_ser); //Initialize the object with communication serial
    void shutdown_arm(); //Initialize shutdown procedure
    void enableTorqueAll(); //Enable torque on all motors
    void disableTorqueAll(); //Disable torque on all motors
    void enableTorqueOne(uint8_t id); //Enable torque on one motor
    void disableTorqueOne(uint8_t id); //Disable torque on one motor
    void setShadowID(uint8_t id, uint8_t sha_id); //Set a shadow id for one motor
    void setExtremePositions(uint8_t id, uint16_t expos[]); //Set the extremum positions for a motor

    void grip(bool gripper=true); //Operate the gripper
    void move_joint(uint8_t id, float theta, char type='R', char controlType='P'); //Move one joint of the robot
    void move_joints(float theta1, float theta2, float theta3, char type='R', char controlType='P'); //Move all joints of the robot
    uint32_t checkPos(uint8_t id); //Return the position of a motor

    void setProfileVel(uint8_t id, uint16_t val); //Set the profile velocity for a motor
    void setProfileAcc(uint8_t id, uint16_t val); //Set the profile Acceleration for a motor
    void setMaxvel(uint8_t id, uint16_t val); //Set the maximum velocity for a motor
    void setMaxAcc(uint8_t id, uint16_t val); //Set the maximum acceleration for a motor
    void setPGain(uint8_t id, uint16_t val); //Set the P gain for one motor
    void setPGainAll(uint16_t val); // Set the P gain for all motors
    void setIGain(uint8_t id, uint16_t val); // Set the I gain for one motor
    void setIGainAll(uint16_t val); // Set the I gain for all motors
    void setDGain(uint8_t id, uint16_t val); // Set the D gain for one motor
    void setDGainAll(uint16_t val); // Set the D gain for all motors

private:
    void _clearBuffer(); //Clear the RX buffer
    void _statusPacket(uint16_t dataLength); //Store the information from the buffer into a storage array
    bool _verifyChecksum(uint16_t dataLength); // Verify the size of the incoming data
    uint16_t _update_crc(uint8_t *arr, uint16_t length);
};

#endif //CRUSTCRAWLER_H
