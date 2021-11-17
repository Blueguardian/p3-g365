#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>
#include "library.h"

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#define DEBUG_SERIAL Serial3
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
#define DEBUG_SERIAL SerialUSB
#else
#define DEBUG_SERIAL Serial
#endif


const uint8_t DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

using namespace ControlTableItem;

char input = 0;
int init_arm = 0;
int motor_def = 1;
bool gripper = false;
CrustCrawler crst;

void setup() {
  input = 0;
  DEBUG_SERIAL.begin(115200); // For mega debugging

  //Set baudrate to 57600 bps, Matching Dynamixel baudrate
  dxl.begin(57600);
  //Set Port protocol version to match with Dynamixel protocol
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  CrustCrawler crst = CrustCrawler();

  for (int i = 1; i <= 5; i++) {
    DEBUG_SERIAL.print("ID: ");
    DEBUG_SERIAL.print(i);
    DEBUG_SERIAL.print(" ,Protocol: ");
    DEBUG_SERIAL.println(DXL_PROTOCOL_VERSION);
    if (dxl.ping(i) == true) {
      DEBUG_SERIAL.println("Ping succeeded!");
      DEBUG_SERIAL.print("Model number: ");
      DEBUG_SERIAL.println(dxl.getModelNumber(i));
    }
    else {
      DEBUG_SERIAL.println("Ping failed");
    }
  }

  if (DEBUG_SERIAL.available()) {
    while (DEBUG_SERIAL.available()) {
      DEBUG_SERIAL.read();
    }
  }
}

void loop() {
  delay(10);
  DEBUG_SERIAL.println("Looping...");
  if(DEBUG_SERIAL.available() > 0){
    input = DEBUG_SERIAL.read();
  }
  if (input == '1' && init_arm == 0)
  {
    DEBUG_SERIAL.println("Initializing");
    crst.init_arm(dxl, DEBUG_SERIAL);
    init_arm = 1;
    input = 0;
  }
  else if (input == '1' && init_arm == 1) {
    if (motor_def < 3) {
      motor_def++;
    }
    else {
      motor_def = 1;
    }
    input = 0;
    DEBUG_SERIAL.println("Shifting motor");
  }
  else if (input == '3' && init_arm == 1) {
    DEBUG_SERIAL.println("Negative");
    while(input == '3') {
      crst.move_joint(motor_def, dxl.getPresentPosition(motor_def) - 320, 'R');
      if(DEBUG_SERIAL.available() > 0) {
        input = DEBUG_SERIAL.read();
      }
    }

  }
  else if (input == '4' && init_arm == 1) {
    DEBUG_SERIAL.println("Positive");
    while(input == '4') {
      crst.move_joint(motor_def, dxl.getPresentPosition(motor_def) + 30, 'R');
      if(DEBUG_SERIAL.available() > 0) {
        input = DEBUG_SERIAL.read();
      }
    }
  }
  else if (input == '2' && init_arm == 1)
  {
    DEBUG_SERIAL.println("Shutting down arm...");
    crst.shutdown_arm();
    input = 0;
    init_arm = 0;
  }
  else if (input == '5' && init_arm == 1) {
    if (gripper == false) {
      crst.grip(true);
      gripper = true;
      DEBUG_SERIAL.println("Closing gripper");
    }
    else if (gripper == true) {
      DEBUG_SERIAL.println("Opening gripper");
      crst.grip(false);
      gripper = false;
    }
    input = 0;
  }
}
