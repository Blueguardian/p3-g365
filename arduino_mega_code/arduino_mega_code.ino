
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
bool gripper = true;
char seq_close[3] = {'2', '2', '1'};
char seq_input[3] = {'0','0','0'};


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
  if (DEBUG_SERIAL.available() > 0) {
    input = DEBUG_SERIAL.read();
  }
  if (input == '1' && init_arm == 0)
  {
    crst.init_arm(dxl, DEBUG_SERIAL);
    init_arm = 1;
    input = 0;
  }
  else if (input == '5' && init_arm == 1) {

  }
  else if (input == '3' && init_arm == 1) {
    while (input == '3') {
      crst.move_joint(motor_def, crst.checkPos(motor_def) - 30, 'R');
      if (DEBUG_SERIAL.available() > 0) {
        input = DEBUG_SERIAL.read();
      }
    }

  }
  else if (input == '4' && init_arm == 1) {
    while (input == '4') {
      crst.move_joint(motor_def, crst.checkPos(motor_def) + 30, 'R');
      if (DEBUG_SERIAL.available() > 0) {
        input = DEBUG_SERIAL.read();
      }
    }
  }
  else if (input == '2' && init_arm == 1)
  {
    for(int i = 0; i<3; i++) {
      seq_input[i] = '0';
    }
    uint8_t counter = 0;
    bool array_check = false;
    seq_input[0] = '2';
    int i = 1;
    while (!array_check) {
      if (seq_input[0] != seq_close[0] || seq_input[1] != seq_input[1] || seq_input[2] != seq_close[3]) {
        array_check = false;
        }
      else if(seq_input[0] == seq_close[0] && seq_input[1] == seq_input[1] && seq_input[2] == seq_close[3]){
        array_check = true;
        }
       else if(DEBUG_SERIAL.available() > 0) {
          char dataR = DEBUG_SERIAL.read();
          if(dataR == '6') {
          }
          else {
            if(i < 3) {
              seq_input[i] = dataR;
              i++; 
          }
          else {
            break;
            }
          }
        }
        else if(counter == 50) {
          if (motor_def < 3) {
            motor_def++;
            }
          else {
            motor_def = 1;
            }
          input = 0;
          break;
        }
        if(array_check) {
        DEBUG_SERIAL.println("Shutting down arm...");
        crst.shutdown_arm();
        input = 0;
        init_arm = 0;
      }
      else {
        DEBUG_SERIAL.println("Waiting for sequence");
      }
      counter++;
      delay(100);
    }
  }
  else if (input == '1' && init_arm == 1) {
    crst.enableTorqueOne(4);
    crst.enableTorqueOne(5);
    if (gripper == false) {
      while (crst.checkPos(4) < 2000 && crst.checkPos(5) < 2000 && input == '1') {
        crst.move_joint(4, crst.checkPos(4) + 20, 'R');
        crst.move_joint(5, crst.checkPos(5) + 20, 'R');
        if (DEBUG_SERIAL.available() > 0) {
          input = DEBUG_SERIAL.read();
        }
      }
      gripper = true;
    }
    else if (gripper == true) {
      while (crst.checkPos(4) > 1100 && crst.checkPos(5) > 1100 && input == '1') {
        crst.move_joint(4, crst.checkPos(4) - 20, 'R');
        crst.move_joint(5, crst.checkPos(5) - 20, 'R');
        if (DEBUG_SERIAL.available() > 0) {
          input = DEBUG_SERIAL.read();
        }
      }
      gripper = false;
    }
  }
}
