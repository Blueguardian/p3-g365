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
    }
  }
}

void loop() {
  uint8_t input = 0;
  delay(10000);
  input = 1;
  if(input == 1) {
    dxl.torqueOn(2);
    dxl.setGoalPosition(2, 2048, UNIT_RAW);
    while(dxl.getPresentPosition(2) != 2048)
    {
      DEBUG_SERIAL.print("Turning...");
      delay(10);
    } 
    }
  }
