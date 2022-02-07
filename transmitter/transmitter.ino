#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
//#define serial_monitor
const uint64_t address = 0xE9E8F0F0E1LL;   //IMPORTANT: The same as in the receiver 0xE9E8F0F0E1LL |0x0000000000000000~0xFFFFFFFFFFFFFFFF
RF24 radio(3,2); // select CE,CSN pin | 
struct transmition_content {
  byte roll;      
  byte pitch;
  byte throttle;
  byte yaw;
  byte mode;
  };
transmition_content data;//創建一個數據封包(要跟前面struct出來的名字一樣)並命名為data  !!注意封包內容物順序
void ResetData() {
  data.throttle = 0; 
  data.pitch = 127; 
  data.roll = 127;
  data.yaw = 127;
  data.mode = 1;
  }
  
void setup(){
  Serial.begin(9600);
   //Start everything up
  radio.begin();
   //radio.setPALevel(RF24_PA_MAX);  MAX IS DEFAULT NO NEED TO DIAL IT AGAIN
  radio.openWritingPipe(address);
  radio.stopListening(); //start the radio comunication to transmitter 
  ResetData();
  }

int calibratetion(int val, int lower, int middle, int upper, bool reverse){//correct Joystick center and its borders 
  val = constrain(val, lower, upper);
  if ( val < middle )
  val = map(val, lower, middle, 0, 128);
  else
  val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
  }
void loop(){
  // Setting may be required for the correct values of the control levers. 
  data.throttle = calibratetion( analogRead(A2), 250, 524, 1015, false );
  data.roll = calibratetion( analogRead(A3), 12, 524, 1020,false );      // "true" or "false" for servo direction 
  data.pitch = calibratetion( analogRead(A4), 12, 524, 1020, false );     // "true" or "false" for servo direction 
  data.yaw = calibratetion( analogRead(A1), 12, 524, 1020, true );       // "true" or "false" for servo direction 
  data.mode = digitalRead(4);
  radio.write(&data, sizeof(transmition_content));

   #ifdef serial_monitor
    Serial.print("throttle:");
    Serial.print(data.throttle);
    Serial.print(" roll:");
    Serial.print(data.roll);
    Serial.print("yaw:");
    Serial.print(data.yaw);
    Serial.print("pitch:");
    Serial.print(data.pitch);
    Serial.print("mode:");
    Serial.println(digitalRead(4));
    #endif
  }
