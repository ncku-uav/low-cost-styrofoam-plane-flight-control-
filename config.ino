//this is recever code
//baud rate : "9600"
//-------------------------------------------------------------------------------
//if your rc plane type is V tail,please uncomment "define V_tail" below ,else, T tail is default in this config
//#define V_tail 

//--------------------------------------------------------------------------------
const uint64_t address = 0xE9E8F0F0E1LL;//declare communication address , must be same to transmitter!!!!(0x0000000000000000~0xFFFFFFFFFFFFFFFF),in Hexadecimal

int Pitch_Range = 30;//define the maximum desire pitch angle,if it is set to 30 ,then ,your air plane can only maneuver in maximum of -30~30 degree in Pitch angle
int Roll_Range = 30;//define the maximum desire pitch angle,if it is set to 30 ,then ,your air plane can only maneuver in maximum of -30~30 degree in Roll angle

float P_Kp=15;//PID parameters for Pitch PID control
float P_Ki=0;
float P_Kd=0;

float R_Kp=15;//PID parameters for Roll PID control
float R_Ki=0;
float R_Kd=0;

//#define serial_monitor
