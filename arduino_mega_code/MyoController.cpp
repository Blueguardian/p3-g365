#include "MyoController.h"
#include "DynamixelShield.h"
#include "Dynamixel2Arduino.h"
#include "HardwareSerial.h"
/**
 * @brief Constructor - Instantiates MyoController object
 */
MyoController::MyoController()
{
    msgChar=String("");
}

/**
 * @brief Destructor
 */
MyoController::~MyoController()
{

}
bool MyoController::initMyo(HardwareSerial &dxl){
    _pSerial = &dxl;
    return true;
}

bool MyoController::updatePose(){
    if (_pSerial->available())
    {
        storageStr = String("");
        while(_pSerial->available())
        {
            storageStr = storageStr + char(_pSerial->read());
            delay(1);
        }

        msgChar = storageStr;
    }

    if(msgChar.indexOf("rest")>=0)
    {
        current_pose_=rest;
    }
    else if (msgChar.indexOf("fist")>=0)
    {
        current_pose_=fist;
    }
    else if (msgChar.indexOf("waveIn")>=0)
    {
        current_pose_=waveIn;
    }
    else if (msgChar.indexOf("waveOut")>=0)
    {
        current_pose_=waveOut;
    }
    else if (msgChar.indexOf("fingersSpread")>=0)
    {
        current_pose_=fingersSpread;
    }
    else if (msgChar.indexOf("doubleTap")>=0)
    {
        current_pose_=doubleTap;
    }
    else
    {
        current_pose_=unknown;
    }
    return true;
}
Poses MyoController::getCurrentPose(){
    return current_pose_;
}

