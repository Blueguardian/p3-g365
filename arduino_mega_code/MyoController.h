#ifndef MyoController_H
#define MyoController_H

#include "Arduino.h"
#include "DynamixelShield.h"
#include "Dynamixel2Arduino.h"
#include "HardwareSerial.h"

#define DEBUG	0

enum Poses{
    rest,
    fist,
    waveIn,
    waveOut,
    fingersSpread,
    doubleTap,
    unknown
};

class MyoController {
public:

    /* Initialization methods */
    MyoController();
    ~MyoController();
    bool initMyo(HardwareSerial &dxl);
    bool updatePose();
    Poses getCurrentPose();
private:
    Poses current_pose_;
    String storageStr;
    String msgChar;
    HardwareSerial *_pSerial; //Pointer to Dynamixel serial
};

#endif