

//
//                       _oo0oo_
//                      o8888888o
//                      88" . "88
//                      (| -_- |)
//                      0\  =  /0
//                    ___/`---'\___
//                  .' \\|     |// '.
//                 / \\|||  :  |||// \
//                / _||||| -:- |||||- \
//               |   | \\\  -  /// |   |
//               | \_|  ''\---/''  |_/ |
//               \  .-\__  '-'  ___/-. /
//             ___'. .'  /--.--\  `. .'___
//          ."" '<  `.___\_<|>_/___.' >' "".
//         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
//         \  \ `_.   \_ __\ /__ _/   .-` /  /
//     =====`-.____`.___ \_____/___.-`___.-'=====
//                       `=---='
//
//
//     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//               
//              佛祖保佑         永無BUG
//
//
//           3               10
//           11              9
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "sensor.h"
#include "ahrs.h"
#include "Control_PID.h"
#include "Bluetooth.h"

//volatile unsigned int count = 0;

void setup() {
  Serial.begin(115200);//38400
  // Serial.setTimeout(50);
  Wire.begin();
  digitalWrite(13, HIGH);
  delay(20);
  mpu6050_initialize();
  delay(20);
  for (uint8_t i = 0; i < 100; i++)
  {
    mpu6050_Gyro_Values();
    mpu6050_Accel_Values();
    delay(20);
  }
  digitalWrite(13, LOW);
  sensor_Calibrate();
  mpu6050_Get_accel();
  mpu6050_Get_gyro();
  ahrs_initialize();//ahrs.h

  delay(10);
  pinMode(3, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  sensorPreviousTime = micros();
  previousTime = micros();
  /*
   TCCR0A = 0x00;

    TCCR0B &= ~_BV(CS12);         // no prescaling
    TCCR0B &= ~_BV(CS11);
    TCCR0B |= _BV(CS10);
    TIMSK1 |= _BV(TOIE1);         // enable timer overflow interrupt
    TCNT1 = 0;
   */
}


void loop()
{
  Dt_sensor = micros() - sensorPreviousTime;
  if (Dt_sensor <= 0) {
    Dt_sensor = 1001;
  }
  // Read data (not faster then every 1 ms)
  if (Dt_sensor >= 1000 && gyroSamples < 3) // = 2760 us
  {
    sensorPreviousTime = micros();
    mpu6050_readGyroSum();
    mpu6050_readAccelSum();
  }
  // 100 Hz task loop (10 ms)  , 5000 = 0.02626 ms
  Dt_roop = micros() - previousTime;
  if (Dt_roop <= 0) {
    Dt_roop = 10001;
  }
  if (Dt_roop >= 10000)
  {
    previousTime = micros();
    G_Dt = Dt_roop / 1000000.0;
    frameCounter++;
    //Read sensor
    mpu6050_Get_gyro();

    if (frameCounter % TASK_100HZ == 0)// 50 Hz tak (20 ms)
    {
      mpu6050_Get_accel();
      GyroXf = ( GyroX2 + GyroX) / 2.0;
      GyroYf = ( GyroY2 + GyroY) / 2.0;
      GyroZf = ( GyroZ2 + GyroZ) / 2.0;
      GyroX2 = GyroX;
      GyroY2 = GyroY;
      GyroZ2 = GyroZ;
      AccXf = AccXf + (AccX - AccXf) * 0.121; //12.4  //Low pass filter ,smoothing factor  α := dt / (RC + dt)
      AccYf = AccYf + (AccY - AccYf) * 0.121; //12.4
      AccZf = AccZf + (AccZ - AccZf) * 0.121; //12.4
      ahrs_updateMARG(GyroXf, GyroYf, GyroZf, AccXf, AccYf, AccZf, G_Dt);
      x_angle = ahrs_r * RAD_TO_DEG;
      y_angle = ahrs_p * RAD_TO_DEG;
      //PID Control///////////
      Control_PIDRate();
      //end PID Control///////////
      //////Out motor///////////
      //u4_yaw=0;
      if(cmmd[5]<25)
      {
      analogWrite(3, 0);//
      analogWrite(9, 0);//
      analogWrite(10, 0);//
      analogWrite(11, 0);//
        }else{
      analogWrite(3, constrain( cmmd[5] + u3_pitch - u2_roll+ u4_yaw, 0, 255 ));//
      analogWrite(9, constrain( cmmd[5] - u3_pitch + u2_roll+ u4_yaw, 0, 255 ));//
      analogWrite(10, constrain(cmmd[5] - u3_pitch - u2_roll- u4_yaw, 0, 255 ));//
      analogWrite(11, constrain(cmmd[5] + u3_pitch + u2_roll- u4_yaw, 0, 255 ));//
        } 
    }
 
    if (frameCounter % TASK_50HZ == 0)// 50 Hz tak (20 ms)
    {
     bluetooth();

       sensorValue = analogRead(A6);
     valt = sensorValue/4;
     Serial.write(valt);

    }
  }
 
}

  /*
  ISR (TIMER1_OVF_vect)
  {
      count++;
      if (count == 24) {
      bluetooth();
      count = 0;
      }
 }
  */
