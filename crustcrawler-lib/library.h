#include <cmath>
#include <arduino.h>
#include <Dynamixel2Arduino.h>
#include <DynamixelShield.h>
#include <cstdint>

#ifndef CRUSTCRAWLER_H
#define CRUSTCRAWLER_H

#define BUF_SIZE 40 //Buffer size

class CrustCrawler {
private:
    const uint8_t _EXPT_NUM_SERVOS = 5; //Expected number of servos available
    const uint8_t _SHA_ID_GRIP = 0x00; //To be determined //Gripper shadow ID for motor four and five
    const uint8_t _ID_ARR[4] = { 1, 2, 3, _SHA_ID_GRIP}; //Array of motor IDs
    const uint16_t _ID_ONE_EXPOS[4] = {0, 0, 0, 4096}; //To be determined //Extremum positions of motor one
    const uint16_t _ID_TWO_EXPOS[4] = {0, 0, 0, 4096}; //To be determined //Extremum positions of motor two
    const uint16_t _ID_THREE_EXPOS[4] = {0, 0, 0, 4096}; //To be determined //Extremum positions of motor three
    const uint16_t _ID_FOUR_EXPOS[4] = {0, 0, 0, 4096}; //To be determined //Extremum positions of motor four (gripper 1)
    const uint16_t _ID_FIVE_EXPOS[5] = {0, 0, 0, 4096}; //To be determined //Extremum positions of motor five (gripper 2)

public:
    CrustCrawler(); //Standard constructor

    void init_arm(HardwareSerial &dxl_ser, uint8_t _controlPin); //Initialize the object with communication serial
    void shutdown_arm();
    void enableTorqueAll(bool enable); //Enable torque on all motors
    void disableTorqueAll(bool disable); //Disable torque on all motors
    void enableTorqueOne(bool enable, uint8_t id); //Enable torque on one motor
    void disableTorqueOne(bool disable, uint8_t id); //Disable torque on one motor
    void setShadowID(uint8_t id, uint8_t sha_id); //Set a shadow id for one motor
    void setExtremePositions(uint8_t id, uint16_t expos[]); //Set the extremum positions for a motor

    void close_grip(bool close=true); //Close the gripper
    void open_grip(bool open=true); //Open the gripper
    void move_joint(uint8_t id, uint16_t theta, char type='R'); //Move one joint of the robot
    void move_joints(uint16_t theta1, uint16_t theta2, uint16_t theta3, bool grip_open=0, char type='R'); //Move all joints of the robot
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
    void _verifyChecksum(uint16_t dataLength); // Verify the size of the incomming data

    HardwareSerial *_pSerial;
};




#endif //CRUSTCRAWLER_H
