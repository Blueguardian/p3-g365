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

int input = 0;
int init_arm = 0;
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
}

void loop() {
    delay(100);
    input = 0;
    
if(DEBUG_SERIAL.available() > 0) {
  input = DEBUG_SERIAL.read();
}
else {
  DEBUG_SERIAL.println("Nothing on line");
}
if(input == '1' && init_arm == 0)
{
  crst.init_arm(dxl, DEBUG_SERIAL);
  input = 0;
  init_arm = 1;
}
else if(input == '2' && init_arm == 1)
{
  crst.shutdown_arm();
  input = 0;
  init_arm = 0;
}


}
