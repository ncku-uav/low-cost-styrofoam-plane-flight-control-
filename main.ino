#include <SCoop.h>//for multi thread
#include <SPI.h>//serial serphirical interface for comunication to nRF24L01
#include <Wire.h>//for I2C communication to mpu6050
#include <RF24.h>//for radio module
#include <MPU6050_tockn.h>//for gyro
#include <Servo.h>//for servo 
#include <PID_v1.h>//for PID control,please change PID output range to -500,500 in header file ,PID_v1.cpp
#include <Time.h>
#include <printf.h>
#include <Smoothed.h>//for data filter

//int status;
int ch_width_1 = 0,ch_width_2 = 0,ch_width_3 = 0,ch_width_4 = 0;//create pwm signal  
int ch_width_v1 =0 , ch_width_v2 =0 ,ch_width_v3 = 0,ch_width_v4 = 0;
int V_ratio=200;

Smoothed <float> imu_pitch_reading; 
Smoothed <float> imu_roll_reading;
Smoothed <float> pwm_pitch_output; 
Smoothed <float> pwm_roll_output;
Smoothed <float> pwm_throttle_output;
Smoothed <float> pwm_yaw_output;

Servo ch1,ch2,ch3,ch4;//define each servo ,here we use the same sequence as general RC plane receiver channel
struct transmition_content {
  byte roll;      
  byte pitch;
  byte throttle;
  byte yaw;
  byte flight_mode;
};//caution !!!! semicolon 
transmition_content data;
RF24 radio(3, 2);//*declare NRF24l01 module(CE,CSN),CE for Chip Enable TX/RX; CSN for Chip Select Node*/
MPU6050 mpu6050(Wire);//*declare IMU module */
unsigned long LastRecvTime = 0; //declare a variable in determine signal loss perpous */

double IMU_Pitch_Reading;       // read pitch from mpu6050
double Pitch_Output ;           //signal to servo;
double Desire_Pitch_Angle ;     //(target angle)received from transmitter 

double IMU_Roll_Reading;        // read pitch from mpu6050
double Roll_Output ;            //signal to servo
double Desire_Roll_Angle ;      //(target angle)received from transmitter 

 
//*declare 2 PID function,for roll and pitch*/
PID Pitch_PID (&IMU_Pitch_Reading, &Pitch_Output, &Desire_Pitch_Angle, P_Kp, P_Ki, P_Kd, DIRECT);
PID Roll_PID (&IMU_Roll_Reading, &Roll_Output, &Desire_Roll_Angle, R_Kp, R_Ki, R_Kd, DIRECT);
 
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  //define each channel to corresponding pin on Arduino nano
  ch1.attach(5);//roll
  ch2.attach(6);//pitch in T tail config/ left tail in V tail config
  ch3.attach(7);//throttle
  ch4.attach(8);//yaw in T tail config / right tail in V tail config
  mySCoop.start();
}
void ReceiveData(){
  while ( radio.available() ) {
  radio.read(&data, sizeof(transmition_content));
  LastRecvTime = millis();
  }
}
void ResetData(){
  data.throttle = 0; 
  data.pitch = 127; 
  data.roll = 127;  
  data.yaw = 127;   
  data.flight_mode=1;
  }

defineTask(TaskTest1);//create thread 1
defineTask(TaskTest2);//create thread 2
//-----------------------------------thread 1 is for communication module,and other process which has nothing to do with PID control -------------------------------//

      void TaskTest1::setup(){    //set up IMU , NRF24l01 , Data Filter //
            Wire.begin();        //activate I2C protocol in used of communicaion between IMU and Arduino Nano
            mpu6050.begin();     //initiate mpu6050
             // mpu6050.calcGyroOffsets(true);
            mpu6050.setGyroOffsets(0.89,0.99,0.56); //to measure gyro offset ,in this process do not move mpu6050
            
            ResetData();        //reset data
            radio.begin();      //activate NRF24l01 module
            radio.openReadingPipe(1,address);//set listening address and flight_mode
            radio.startListening();
          
            pwm_throttle_output.begin(SMOOTHED_EXPONENTIAL, 10);  
            pwm_yaw_output.begin(SMOOTHED_EXPONENTIAL, 10);
            pwm_pitch_output.clear();  
            pwm_roll_output.clear();
             }
      void TaskTest1::loop(){
        ReceiveData();//start execute the function of receving data from transmitter
        unsigned long now = millis();
        if ( now - LastRecvTime > 1000 ){//determine whether we lost signal
          ResetData(); //Reset data to ensure all control-surface position turn to neutral
            digitalWrite(LED_BUILTIN, HIGH); 
            //Serial.println("signal loss");
        }
        else{
          digitalWrite(LED_BUILTIN, LOW); 
           //Serial.println("connected");
        }
        ch_width_1 = map(data.roll, 0, 255, 1000, 2000);
        ch_width_2 = map(data.pitch, 0, 255, 1000, 2000);
        ch_width_3 = map(data.throttle, 0, 255, 1000, 2000);    // pin D7 (PWM signal)
        pwm_throttle_output.add(ch_width_3);
        ch_width_v3=pwm_throttle_output.get();
        ch3.writeMicroseconds(ch_width_v3);
        //-----------------------------------------------
        #ifdef V_tail   //------------------V_tail config-------------------------------------------------//
          ch_width_4 = map(data.yaw,0, 255, -V_ratio, V_ratio); // pin D8 (PWM signal)
          pwm_yaw_output.add(ch_width_4);
        #else    //------------------T_tail config-------------------------------------------------//
          ch_width_4 = map(data.yaw, 0, 255, 1000, 2000);
          pwm_yaw_output.add(ch_width_4);
          ch_width_v4 = pwm_yaw_output.get();
          ch4.writeMicroseconds(ch_width_v4);
        #endif
      }
//---------------------------------------------thread 2 is for PID control in pitch and roll-------------------------------------------//
      void TaskTest2::setup(){
        imu_pitch_reading.begin(SMOOTHED_AVERAGE, 10);  
        imu_roll_reading.begin(SMOOTHED_AVERAGE, 10); 
        pwm_pitch_output.begin(SMOOTHED_AVERAGE, 10);  
        pwm_roll_output.begin(SMOOTHED_AVERAGE, 10);
        imu_pitch_reading.clear();
        imu_roll_reading.clear();
        pwm_pitch_output.clear();
        pwm_roll_output.clear();
      
        
        Desire_Pitch_Angle = 0;
        //Turn the PID on
        Pitch_PID.SetMode(AUTOMATIC);
        //Adjust PID values
        Pitch_PID.SetTunings(P_Kp, P_Ki, P_Kd);
        
        Desire_Roll_Angle = 0;
        //Turn the PID on
        Roll_PID.SetMode(AUTOMATIC);
        //Adjust PID values
        Roll_PID.SetTunings(R_Kp, R_Ki, R_Kd);
      }
      void TaskTest2::loop(){
          mpu6050.update();//Refresh mpu data 
      
          imu_pitch_reading.add(mpu6050.getAngleX());  //read gyro data into filter
          imu_roll_reading.add(mpu6050.getAngleY());   //read gyro data into filter
           
          //--------------------------------Pitch ,Roll PID control--------------------------------------------------------------------------------------------------------------------
            Desire_Pitch_Angle= map(data.pitch, 0, 255, -Pitch_Range, Pitch_Range ); //convert joystick's analog signal to angle , to unitize data to be compute in PID    
            IMU_Pitch_Reading = imu_pitch_reading.get();
            Pitch_PID.Compute();  //PID calculation
            
            Desire_Roll_Angle= map(data.roll, 0, 255, -Roll_Range, Roll_Range );
            IMU_Roll_Reading = imu_roll_reading.get(); 
            Roll_PID.Compute();  //PID calculation
            
          //--------------------------------Push the result into output pin to servo for which PWM signal is from PID computation------------------------------------------------
          if(data.flight_mode==1){ch_width_v1 = Roll_Output+1500;}
          else{ ch_width_v1=ch_width_1;}
          pwm_roll_output.add(ch_width_v1);
          ch_width_v1 = pwm_roll_output.get();
          ch1.writeMicroseconds(ch_width_v1);
          //-------------------------------Convert PWM signal to correlative servo pin fot V Tail ,and T tail------------------------------------------------------------------
          #ifdef V_tail //------------------V_tail config-------------------------------------------------//
              if(data.flight_mode==1){          //---"flight_mode == 1" indicate stabilized is activated---// 
                    ch_width_v2=ch_width_4+(Pitch_Output+1500);
                    ch_width_v4=3000-ch_width_v2;
              }
              else{                      //---"flight_mode != 1" indicate manual flight mode---//
                    ch_width_v2=ch_width_4+ch_width_2;
                    ch_width_v4=3000-ch_width_v2;
              }
              pwm_pitch_output.add(ch_width_v2);
              ch_width_v2 = pwm_pitch_output.get(); 
                ch2.writeMicroseconds(ch_width_v2);
                ch4.writeMicroseconds(3000-ch_width_v2);
          #else//----------------------------T_tail config-----------------------------------------//
              if(data.flight_mode==1){ch_width_v2=Pitch_Output+1500;}
              else{ ch_width_v2=ch_width_2;}
              pwm_pitch_output.add(ch_width_v2);
              ch_width_v2 = pwm_pitch_output.get();
              ch2.writeMicroseconds(ch_width_v2);           
          #endif
          //-------------------------------serial_monitor to monitor each servo channel in pwm signal ---------------------------------------------------------------------------
          //--------------please cancel this function with comment "define serial_monitor" in config ,if you don't need it ,or it will slow down arduino-------------------------
          #ifdef serial_monitor
              Serial.print("Pitch:");  
              Serial.print(ch_width_v2);
              Serial.print("  Roll:");
              Serial.print(ch_width_v1);
              Serial.print("  throttle:");
              Serial.print(ch_width_3);
              Serial.print("  yaw:");
              Serial.println(ch_width_v4);
          #endif
       }
void loop(){
  yield();
}
