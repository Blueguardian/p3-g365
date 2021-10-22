#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>

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

void grip_close() {
  dxl.torqueOn(4);
  dxl.torqueOn(5);
  dxl.setGoalPosition(4, 2000, UNIT_RAW);
  dxl.setGoalPosition(5, 2000, UNIT_RAW);
  while(!(dxl.getPresentPosition(4) > 1990 && dxl.getPresentPosition(4) < 2010 && dxl.getPresentPosition(5) > 1990 && dxl.getPresentPosition(5) < 2010)) {
    DEBUG_SERIAL.println("Gripper not open");
    DEBUG_SERIAL.print("Motor 4 present position: ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(4));
    DEBUG_SERIAL.print("Motor 5 present position: ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(5));
    
  }
  delay(1000);
  dxl.torqueOff(4);
  dxl.torqueOff(5);
}

void grip_open() {
  dxl.torqueOn(4);
  dxl.torqueOn(5);
  dxl.setGoalPosition(4, 1100, UNIT_RAW);
  dxl.setGoalPosition(5, 1100, UNIT_RAW);
  while(!(dxl.getPresentPosition(4) > 1090 && dxl.getPresentPosition(4) < 1110 && dxl.getPresentPosition(5) > 1090 && dxl.getPresentPosition(5) < 1110)) {
    DEBUG_SERIAL.println("Gripper not closed");
    DEBUG_SERIAL.print("Motor 4 present position: ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(4));
    DEBUG_SERIAL.print("Motor 5 present position: ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(5));
    
  }
  delay(1000); 
  dxl.torqueOff(4);
  dxl.torqueOff(5);
}

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
if(input == '1')
{
    dxl.torqueOn(2);
    dxl.torqueOn(1);
    dxl.torqueOn(3);
    dxl.setGoalPosition(2, 2048, UNIT_RAW);
    while(!(dxl.getPresentPosition(2) < 2053 && dxl.getPresentPosition(2) > 2038))
    {
      Serial1.print("Motor 2 position: ");
      Serial1.println(dxl.getPresentPosition(2));
      delay(10);
    }
    delay(100);
    dxl.setGoalPosition(3, 1024, UNIT_RAW);
    while(!(dxl.getPresentPosition(3) < 1029 && dxl.getPresentPosition(3) > 1019)) {
      Serial1.print("Motor 3 position   ");
      Serial1.println(dxl.getPresentPosition(3));
      delay(10);
    }
    delay(100);
    dxl.setGoalPosition(1, 1024, UNIT_RAW);
    while(!(dxl.getPresentPosition(1) < 1029 && dxl.getPresentPosition(1) > 1019)) {
      Serial1.print("Motor 1 position   ");
      Serial1.println(dxl.getPresentPosition(1));
      
      delay(10);
    }
    grip_open();
    delay(100);
  grip_close();
  delay(100);
  input = 0;
}
else if(input == '2')
{
      dxl.torqueOn(2);
      dxl.torqueOn(1);
      dxl.torqueOn(3);
      dxl.setGoalPosition(2, 2048, UNIT_RAW);
    while(!(dxl.getPresentPosition(2) < 2053 && dxl.getPresentPosition(2) > 2038))
    {
      DEBUG_SERIAL.print("Motor 2 position: ");
      Serial1.println(dxl.getPresentPosition(2));
      delay(10);
    }
    delay(100);
    dxl.setGoalPosition(3, 1024, UNIT_RAW);
    while(!(dxl.getPresentPosition(3) < 1034 && dxl.getPresentPosition(3) > 1014)) {
      Serial1.print("Motor 3 position   ");
      Serial1.println(dxl.getPresentPosition(3));
      delay(10);
    }
    dxl.setGoalPosition(1, 512, UNIT_RAW);
    while(!(dxl.getPresentPosition(1) < 517 && dxl.getPresentPosition(1) > 507)) {
      Serial1.print("Motor 1 position: ");
      Serial1.println(dxl.getPresentPosition(1));
      delay(10);
    }
    dxl.setGoalPosition(2, 1174, UNIT_RAW);
    while(!(dxl.getPresentPosition(2) < 1179 && dxl.getPresentPosition(2) > 1169)) {
      Serial1.print("Motor 2 position: ");
      Serial1.println(dxl.getPresentPosition(1));
      delay(10);
    }
  delay(100);
  dxl.torqueOff(2);
  dxl.torqueOff(1);
  dxl.torqueOff(3);
  input = 0;
}


}
