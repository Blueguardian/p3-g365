#include <Dynamixel2Arduino.h>
#include <DynamixelShield.h>
#include <SoftwareSerial.h>

#define DXL_SERIAL   Serial
#define DEBUG_SERIAL soft_serial

const uint8_t dxl_dir_pin = 2; // DYNAMIXEL Shield DIR PIN

SoftwareSerial soft_serial(19, 18); // DYNAMIXELShield UART RX/TX
Dynamixel2Arduino dxl(DXL_SERIAL, dxl_dir_pin);

#define TORQUE_ENABLE_ADDR          64
#define TORQUE_ENABLE_ADDR_LEN      1
#define LED_ADDR                    25
#define LED_ADDR_LEN                1
#define GOAL_POSITION_ADDR          116
#define GOAL_POSITION_ADDR_LEN      2
#define PRESENT_POSITION_ADDR       132
#define PRESENT_POSITION_ADDR_LEN   4
#define TIMEOUT 10

const uint8_t turn_on = 1;
const uint8_t turn_off = 0;

void setup() {
  DEBUG_SERIAL.begin(115700);
  while(!DEBUG_SERIAL);

  dxl.begin(57600);
 //TODO: Setup pins for IO_serial
 //IO_serial.begin(1157000);
  
  DYNAMIXEL::InfoFromPing_t ping_info[32];
  while(uint8_t count = dxl.ping(0xFE, ping_info, sizeof(ping_info)/sizeof(ping_info[0])) < 5) {
    DEBUG_SERIAL.print("Crustcrawler only found: ");
    DEBUG_SERIAL.print(count);
    DEBUG_SERIAL.println(" motors, retrying...");
  }
  /*For debugging purposes:
   * Print motor data
   */
  DYNAMIXEL::InfoFromPing_t ping_motor[32];
  if(uint8_t count_ping = dxl.ping(0xFE, ping_motor, sizeof(ping_motor)/sizeof(ping_motor[0])))
  {
    DEBUG_SERIAL.print("Detected motor : \n");
    for(int i; i < count_ping; i++) {
      DEBUG_SERIAL.print("    ");
      DEBUG_SERIAL.print(ping_motor[i].id, DEC);
      DEBUG_SERIAL.print(", Model: ");
      DEBUG_SERIAL.print(ping_motor[i].model_number);
      DEBUG_SERIAL.print(", Ver: ");
      DEBUG_SERIAL.println(ping_motor[i].firmware_version, DEC);
      delay(1000);
    }
  }
}

void loop() {
  char input = 0;
  delay(10000);
  input = 1;
  if(input == 1) {
    dxl.torqueOn(2);
    dxl.torqueOn(1);
    dxl.torqueOn(3);
    dxl.writeControlTableItem((uint8_t)44, (uint8_t)1, (uint32_t)10, (uint32_t)10);
    dxl.writeControlTableItem((uint8_t)44, (uint8_t)2, (uint32_t)10, (uint32_t)10);
    dxl.writeControlTableItem((uint8_t)44, (uint8_t)3, (uint32_t)10, (uint32_t)10);
    dxl.writeControlTableItem((uint8_t)36, (uint8_t)1, (uint32_t)200, (uint32_t)10);
    dxl.writeControlTableItem((uint8_t)36, (uint8_t)2, (uint32_t)200, (uint32_t)10);
    dxl.writeControlTableItem((uint8_t)36, (uint8_t)3, (uint32_t)200, (uint32_t)10);
    dxl.setGoalPosition(2, 2048, UNIT_RAW);
    while(!(dxl.getPresentPosition(2) < 2053 && dxl.getPresentPosition(2) > 2043))
    {
      DEBUG_SERIAL.print("Motor 2 position: ");
      DEBUG_SERIAL.println(dxl.getPresentPosition(2));
      delay(10);
    }
    delay(100);
    dxl.setGoalPosition(3, 1024, UNIT_RAW);
    while(!(dxl.getPresentPosition(3) < 1029 && dxl.getPresentPosition(3) > 1019)) {
      DEBUG_SERIAL.print("Motor 3 position   ");
      DEBUG_SERIAL.println(dxl.getPresentPosition(3));
      delay(10);
    }
    delay(100);
    dxl.setGoalPosition(1, 1024, UNIT_RAW);
    while(!(dxl.getPresentPosition(1) < 1029 && dxl.getPresentPosition(1) > 1019)) {
      DEBUG_SERIAL.print("Motor 1 position   ");
      DEBUG_SERIAL.println(dxl.getPresentPosition(1));
      
      delay(10);
    }
    delay(10000);
    }
    delay(100);
    input = 2;
    if(input == 2) {
      dxl.torqueOn(2);
      dxl.torqueOn(1);
      dxl.torqueOn(3);
      dxl.writeControlTableItem((uint8_t)44, (uint8_t)1, (uint32_t)10, (uint32_t)10);
      dxl.writeControlTableItem((uint8_t)44, (uint8_t)2, (uint32_t)10, (uint32_t)10);
      dxl.writeControlTableItem((uint8_t)44, (uint8_t)3, (uint32_t)10, (uint32_t)10);
      dxl.writeControlTableItem((uint8_t)36, (uint8_t)1, (uint32_t)200, (uint32_t)10);
      dxl.writeControlTableItem((uint8_t)36, (uint8_t)2, (uint32_t)200, (uint32_t)10);
      dxl.writeControlTableItem((uint8_t)36, (uint8_t)3, (uint32_t)200, (uint32_t)10);
      dxl.setGoalPosition(2, 2048, UNIT_RAW);
    while(!(dxl.getPresentPosition(2) < 2053 && dxl.getPresentPosition(2) > 2043))
    {
      DEBUG_SERIAL.print("Motor 2 position: ");
      DEBUG_SERIAL.println(dxl.getPresentPosition(2));
      delay(10);
    }
    delay(100);
    dxl.setGoalPosition(3, 1024, UNIT_RAW);
    while(!(dxl.getPresentPosition(3) < 1034 && dxl.getPresentPosition(3) > 1014)) {
      DEBUG_SERIAL.print("Motor 3 position   ");
      DEBUG_SERIAL.println(dxl.getPresentPosition(3));
      delay(10);
    }
    dxl.setGoalPosition(1, 512, UNIT_RAW);
    while(!(dxl.getPresentPosition(1) < 517 && dxl.getPresentPosition(1) > 507)) {
      DEBUG_SERIAL.print("Motor 1 position: ");
      DEBUG_SERIAL.println(dxl.getPresentPosition(1));
      delay(10);
    }
    dxl.setGoalPosition(2, 1174, UNIT_RAW);
    while(!(dxl.getPresentPosition(2) < 1179 && dxl.getPresentPosition(2) > 1169)) {
      DEBUG_SERIAL.print("Motor 2 position: ");
      DEBUG_SERIAL.println(dxl.getPresentPosition(1));
      delay(10);
    }
    delay(10000);
      dxl.torqueOff(2);
      dxl.torqueOff(1);
      dxl.torqueOff(3);
    }
  }
