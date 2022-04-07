//40 140
float roll_I_rate;
float roll_D_rate;
float err_roll_rate;
float err_roll_ant_rate;

float pitch_I_rate;
float pitch_D_rate;
float err_pitch_rate;
float err_pitch_ant_rate;

float yaw_I_rate;
float yaw_D_rate;
float err_yaw_rate = 0;
float err_yaw_ant_rate = 0;
float delta=0,lastGyro=0,deltaSum=0,delta1,delta2,delta3; 
float delta_pitch=0,lastGyro_pitch=0,deltaSum_pitch=0,delta1_pitch,delta2_pitch,delta3_pitch; 
//PID mode control
int u2_roll = 0.0;
int u3_pitch = 0.0;
int u4_yaw = 0.0;

// 2.5  0.4  0.35
float ratep = 1.3, ratei = 0, rated =0.2,ratea=0.15;
float ratep_roll =1.3 , ratei_roll  = 0, rated_roll  =0.2,ratea_roll=0.15;

float ratep_yaw=148 , ratei_yaw=0 , rated_yaw = 0;

int  con_roll, con_pitch, con_yaw;
 
void Control_PIDRate() {

  con_pitch = (cmmd[6] - 90) * 0.5;
  con_roll = (cmmd[7] - 90) * -0.5;
  
  con_yaw = (cmmd[8] - 160) *0.5625;
//thr cmmd[5]   26 41 26
//kp cmmd[6]  21 28 72
//kd cmmd[7]
//ka cmmd[8]
  err_roll_rate = 0- GyroXf * RAD_TO_DEG;

  
    delta          =  err_roll_rate - lastGyro;  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
    lastGyro        = err_roll_rate;
    deltaSum       = delta1+delta2+delta;
    delta2       = delta1;
    delta1        = delta;
 deltaSum=deltaSum;
  
   
     // err_roll_rate = constrain(err_roll_rate, -35, 35);     
  err_roll =    -(con_roll + ahrs_r) * ratep_roll/256*10*kkp   ;
  //  roll_I_rate += err_roll*Ki_rateRoll*G_Dt*ratei_roll*cmmd[2]*0.04 ;
  // if(cmmd[2] ==0)   roll_I_rate=0;
  //roll_I_rate = constrain(roll_I_rate, -50, 50);

  
  u2_roll =  -GyroXf * RAD_TO_DEG* rated_roll*kkd/256*10 /*Kd_rateRoll */+deltaSum*ratea*kka/256*10   /*+ roll_I_rate*/ + err_roll ;
  u2_roll = constrain(u2_roll, -90, 90);//300 //+-400



  // PITCH CONTROL
  err_pitch_rate  = (0 - GyroYf * RAD_TO_DEG)  ;  

  delta          =  err_pitch_rate - lastGyro_pitch;  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
    lastGyro_pitch        = err_pitch_rate;
    deltaSum_pitch       = delta1_pitch+delta2_pitch+delta_pitch;
    delta2_pitch       = delta1_pitch;
    delta1_pitch        = delta_pitch;
   //   err_pitch_rate = constrain(err_pitch_rate, -35, 35);   
  err_pitch = -(con_pitch + ahrs_p) * ratep/256*10*kkp ;  
  //  pitch_I_rate += err_pitch*Ki_ratePitch*G_Dt*ratei*cmmd[2]*0.04;
  // if(cmmd[2]==0)  pitch_I_rate=0;
  // pitch_I_rate = constrain(pitch_I_rate, -20, 20);
  
 if(cmmd[5]>120) kkp= 20*(0.9+((float)cmmd[5]-120)/1350);
  u3_pitch =  - GyroYf * RAD_TO_DEG* rated*kkd/256*10 /*Kd_ratePitch */+deltaSum_pitch*ratea*kka/256*10 /*pitch_I_rate*/  +err_pitch;
  u3_pitch = constrain(u3_pitch, -90, 90);//300 //+-400



  // YAW CONTROL
  float Set_yaw = 0;
  //err_yaw_rate = (Set_yaw - GyroYf * RAD_TO_DEG) * rated_yaw * 250;

  err_yaw = (Set_yaw + Heading +con_yaw )* ratep_yaw/60 ; //215
  
  yaw_I_rate += Heading*Ki_rateYaw*G_Dt*ratei_yaw/100;
  yaw_I_rate = constrain(yaw_I_rate, -30, 30);
  
  u4_yaw =   err_yaw+yaw_I_rate /*- err_yaw_rate*/;
 // u4_yaw = constrain(u4_yaw, -120, 120);//250 +-400
  /*
  Serial.print(  err_yaw); Serial.print("\t");
  Serial.print( -err_yaw_rate); Serial.print("\t");
  Serial.print( u4_yaw); Serial.print("\t");
  Serial.print("\n");
  */
}
