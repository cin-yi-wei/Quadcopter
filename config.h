
//PID-------------Rate
float Kp_rateRoll = 1.18;//1.18 1.200
float Kpi_rateRoll = 1.32;//1.02 1.32
float Ki_rateRoll = 2.75;//2.25  0.25  -  0.985
float Kd_rateRoll = 0.035;//0.035 0.025 - 0.045
float err_roll;



float Kp_ratePitch = 1.18;//1.18 1.200
float Kpi_ratePitch = 1.32;//1.02 1.32
float Ki_ratePitch = 2.75;//2.25 0.5 - 2.8
float Kd_ratePitch = 0.035;//0.025 - 0.045
float err_pitch;



float Kp_rateYaw = 1.75;//1.75 - 3.450  350.0
float Kpi_rateYaw = 0.0;
float Ki_rateYaw = 3.65;//3.65  2.95
float Kd_rateYaw = 0.065;//0.045 0.065
float err_yaw;

//PID--------------Stable
float Kp_levelRoll= 5.2;//6.2 
float Ki_levelRoll= 0.00;//0.0
float Kd_levelRoll= 0.00;//0.0

float Kp_levelPitch= 5.2;//6.2 
float Ki_levelPitch= 0.00;
float Kd_levelPitch= 0.00;

float Kp_levelyaw= 0.0;



// Automatic take-off and landing 
#define h_control 2.2  //0.6 0.9 meter

#define tar 0.01
//Parameter system Quadrotor
#define m_quad 1.1 //kg
#define L_quad 0.175 //m



int A_X_MIN = -4160;    //
int A_X_MAX = 3968;     //
int A_Y_MIN = -4137;    //
int A_Y_MAX = 4063;     //
int A_Z_MIN = -4216;    //
int A_Z_MAX = 4007;     //4007
//magnetometer calibration constants; use the Calibrate example from
// the Pololu library to find the right values for your board




#define TASK_100HZ 1
#define TASK_50HZ 2
#define TASK_20HZ 5
#define TASK_10HZ 10
#define TASK_5HZ 20
#define TASK_1HZ 100
#define RAD_TO_DEG 57.295779513082320876798154814105

  //direction cosine matrix (DCM)   Rotated Frame to Stationary Frame ZYX
float DCM00 = 1.0;
float DCM01 = 0.0;
float DCM02 = 0.0;
float DCM10 = 0.0;
float DCM11 = 1.0;
float DCM12 = 0.0;
float DCM20 = 0.0;
float DCM21 = 0.0;
float DCM22 = 1.0;
  
// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long sensorPreviousTime = 0;
uint8_t frameCounter = 0;
float G_Dt = 0.01; 

long Dt_sensor = 1000;
long Dt_roop = 10000;
int Status_LED = LOW;
int ESC_calibra = 0;
float cos_rollcos_pitch = 1.0;
float accrX_Earth = 0.0;
float accrY_Earth = 0.0;
float accrZ_Earth = 0.0;
float accrZ_Earthf = 0.0;
float x_angle;
float y_angle;


byte cmmd[12],buf,lm=128;
int insize = 0,RW = 0,addr=0;
float kkp=20,kkd=28,kka=0 ;
float sensorValue;
byte valt;

byte cel;
byte reading;





