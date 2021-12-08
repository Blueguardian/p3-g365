//Necessary includes for control of CrustCrawler
#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>
#include "library.h"

//Definition of used Serial depending on the Arduino used
//Shouldn't be necessary but only works using this type
//of definition
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#define DEBUG_SERIAL Serial3
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
#define DEBUG_SERIAL SerialUSB
#else
#define DEBUG_SERIAL Serial
#endif


//Define Dynamixel protocol version used
const uint8_t DXL_PROTOCOL_VERSION = 2.0;

//Define Serial connection as Dynamixel Object.
DynamixelShield dxl;

//Namespace for controltable item names
using namespace ControlTableItem;


//Initialize variables for control
char input = 0;
int init_arm = 0;
int motor_def = 1;
bool gripper = true;
unsigned long time_t;

//Define CrustCrawler object
CrustCrawler crst;

void setup() {
  //Initialize DEBUG_SERIAL for debugging purposes
  DEBUG_SERIAL.begin(115200);

  //Set baudrate to 57600 bps this matches the standard baudrate of Dynamixel
  dxl.begin(57600);
  //Set Port protocol version to match with Dynamixel protocol
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  //Instantiate CrustCrawler Object
  CrustCrawler crst = CrustCrawler();

  //Check for availability of servos
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
  //Empty the buffer, if data is available
  if (DEBUG_SERIAL.available()) {
    while (DEBUG_SERIAL.available()) {
      DEBUG_SERIAL.read();
    }
  }
  //Initialize the timer
  time_t = millis();
}

//Main program loop
void loop() {
  //Check for avaiable data, and assign it to input if available
  if (DEBUG_SERIAL.available() > 0) {
    input = DEBUG_SERIAL.read();
  }
  //Create a sampling time, here given as 10 ms
  //Runs only when 10 ms has passed since last check
  if(millis()-time_t>=10) {

    //If the user inputs a "Fist" and the arm is not initialized
    if (input == '1' && init_arm == 0)
    {
      //Initialize the arm
      crst.init_arm(dxl, DEBUG_SERIAL);
      //Assign the initialization variable to 1 and reset the input
      init_arm = 1;
      input = 0;
    }
    //If the user inputs fist and the arm is initialized
    else if (input == '1' && init_arm == 1) {

      //Open the gripper 
      if (gripper == false) {
        //Initialize another time variable for checking position
        unsigned long init_time_t = millis();

        //Declare and assign position variables
        uint16_t des_pos1 = crst.checkPos(4) + 20;
        uint16_t des_pos2 = crst.checkPos(5) + 20;

        //If the position of the CrustCrawler gripper is not at the maximum position
        //continue to open gripper
        if (crst.checkPos(4) < 2000 && crst.checkPos(5) < 2000 && input == '1') {
          crst.move_joint(4, des_pos1, 'R');
          crst.move_joint(5, des_pos2, 'R');
        }
        //If the CrustCrawler gripper is close to the final position
        //Assign the gripper variable to true
        else if(millis()-init_time_t >= 10 && (crst.checkPos(4) < des_pos1-1 && crst.checkPos(5) < des_pos2-1)) {
          gripper = true;
        }
      }
      //Else if the gripper is open, close the gripper
      else if (gripper == true) {
        unsigned long init_time_t = millis();

        //Declare and assign new position variables
        uint16_t des_pos1 = crst.checkPos(4) - 20;
        uint16_t des_pos2 = crst.checkPos(5) - 20;

        //while the CrustCrawler is not at the closed end position continue
        //to close the gripper
        if (crst.checkPos(4) > 1100 && crst.checkPos(5) > 1100 && input == '1') {
          crst.move_joint(4, crst.checkPos(4) - 20, 'R');
          crst.move_joint(5, crst.checkPos(5) - 20, 'R');
        }
        //If CrustCrawler is close to end position of closed gripper
        //Change gripper variable
        else if((millis()-init_time_t >= 10 && (crst.checkPos(4) > des_pos1+1 && crst.checkPos(5) > des_pos2+1))) {
        gripper = false;
        }
      }
    }
    //If the user inputs "FingersSpread" change the controlled motor incrementally
    if (input == '2' && init_arm == 1) {
      if (motor_def < 3) {
        motor_def++;
      }
      else {
        //If the motor value is above 3 set the motor value to 1 instead
        motor_def = 1;
      }
    }
    //If the user inputs "Wavein" move the selected motor 30 units clockwise
    else if (input == '3' && init_arm == 1) {
        crst.move_joint(motor_def, crst.checkPos(motor_def) - 30, 'R');
    }
    //If the user inputs "WaveOut" move the selected motor 30 units counter clock wise
    else if (input == '4' && init_arm == 1) {
        crst.move_joint(motor_def, crst.checkPos(motor_def) + 30, 'R');
    }
    
    //If the user inputs the shutdown sequence defined in the python script
    //Shut down the CrustCrawler and reset control variables
    else if (input == '6' && init_arm == 1)
    {
      DEBUG_SERIAL.println("Shutting down arm...");
      crst.shutdown_arm();
      input = 0;
      init_arm = 0;
    }
    //Increment the timer
    time_t += 10;
  }
}
