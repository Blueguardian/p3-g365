#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>
#include "library.h"
#include "MyoController.h"

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #define DEBUG_SERIAL Serial3
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB
#else
  #define DEBUG_SERIAL Serial
#endif


const uint8_t DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;
MyoController myo = MyoController();

using namespace ControlTableItem;

int input = 0;
int init_arm = 0;
int motor_def = 1;
bool gripper = false;
CrustCrawler crst;

void setup() {
  DEBUG_SERIAL.begin(115200); // For mega debugging

  //Set baudrate to 57600 bps, Matching Dynamixel baudrate
  dxl.begin(57600);
  //Set Port protocol version to match with Dynamixel protocol
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for(int i = 1; i <= 5; i++) {
    DEBUG_SERIAL.print("ID: ");
    DEBUG_SERIAL.print(i);
    DEBUG_SERIAL.print(" ,Protocol: ");
    DEBUG_SERIAL.println(DXL_PROTOCOL_VERSION);
    if(dxl.ping(i) == true) {
      DEBUG_SERIAL.println("Ping succeeded!");
      DEBUG_SERIAL.print("Model number: ");
      DEBUG_SERIAL.println(dxl.getModelNumber(i));
    }
    else {
      DEBUG_SERIAL.println("Ping failed");
    }
  }
  CrustCrawler crst = CrustCrawler();
  myo.initMyo();
}

void loop() {
    delay(10);
    input = 0;
    myo.updatePose();
    switch (myo.getCurrentPose()) {
      case rest:
      DEBUG_SERIAL.println("Nothing on line");
      DEBUG_SERIAL.print("Motor: ");
      DEBUG_SERIAL.print(motor_def);
      DEBUG_SERIAL.println(" selected");
      break;
      case fist:
      input = 1;
      break;
      case waveIn:
      input = 2;
      break;
      case waveOut:
      input = 3;
      break;
      case fingersSpread:
      input = 4;
      break;
      case doubleTap:
      input = 5;
      break;    
    }
//if(DEBUG_SERIAL.available() > 0) {
//  input = DEBUG_SERIAL.read();
//}
//else {
//  
//}
if(input == '1' && init_arm == 0)
{
  crst.init_arm(dxl, DEBUG_SERIAL);
  input = 0;
  init_arm = 1;
}
else if(input == '1' && init_arm == 1) {
  if(motor_def < 3) {
    motor_def++;
  }
  else {
    motor_def = 1;
  }
}
else if(input == '2' && init_arm == 1) {
  DEBUG_SERIAL.println("Negative");
  crst.move_joint(motor_def, dxl.getPresentPosition(motor_def)-10, 'R'); 
}
else if(input == '3' && init_arm == 1) {
  DEBUG_SERIAL.println("Positive");
  crst.move_joint(motor_def, dxl.getPresentPosition(motor_def)+10, 'R');
}
else if(input == '4' && init_arm == 1)
{
  crst.shutdown_arm();
  input = 0;
  init_arm = 0;
}
else if( input == '5' && init_arm == 1) {
  if(gripper == false) {
    crst.grip(true);
    gripper = true;
  }
  else if(gripper == true) {
    crst.grip(false);
    gripper = false;
  }
}
}
