// I have been using gobal declaration for most part

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU9250.h"

//servo/ motor delcaration
Servo FirstEsc, SecondEsc, ThirdEsc, FourthEsc;
volatile float PWM_motor_1 = 0,PWM_motor_2 = 0,
               PWM_motor_3 = 0,PWM_motor_4 = 0;
//declaration of gyros value
MPU9250 accelgyro;// calling from MPU9250 to initialise

int16_t ax,ay,az,gx,gy,gz,mx,my,mz = 0;

// first for offset second and third are variable that hold gyro and accel value
float G_cal_x, G_cal_y, G_cal_z = 0;
float Gxyz[3] = {0,0,0};
float Axyz[3] = {0,0,0};

// converting into euler angles
float Roll_gyro = 0, Pitch_gyro = 0;
float Roll_accel = 0, Pitch_accel = 0;
float Roll_output = 0, Pitch_output = 0;

//pid calculation values
float pid_error_temp = 0, pid_i_term_roll = 0, pid_roll_output = 0;
float setpoint_roll = 0, last_er_tmp_roll = 0;
float setpoint_pitch = 0, pid_i_term_pitch = 0;
float Ki_roll = 0.00005, Kp_roll = 0.17, Kd_roll = 1.7;
float pid_pitch_output = 0, last_er_tmp_pitch = 0;

//support for the calculation
float dt = 0, last_tm = 0;
int value = 0;
float pid_max_roll = 300;
float loop_timer = 0;

//function support
void get_gyro();
void get_accel();
void PID_compute();

void setup()
{
  Wire.begin();
  //initialise the clock speed, gyroscope, accel,
  FirstEsc.attach(5);
  SecondEsc.attach(6);
  ThirdEsc.attach(9);
  FourthEsc.attach(10);

  FirstEsc.writeMicroseconds(1000);
  SecondEsc.writeMicroseconds(1000);
  ThirdEsc.writeMicroseconds(1000);
  FourthEsc.writeMicroseconds(1000);

  accelgyro.initialize();
  //serial communication begin
  for(int i = 0; i < 2000; i++)
     {
       get_gyro();
       G_cal_x += Gxyz[0];
       G_cal_y += Gxyz[1];
       G_cal_z += Gxyz[2];
       delay(3);
     }
  G_cal_x /= 2000;
  G_cal_y /= 2000;
  G_cal_z /= 2000;

  Serial.begin(9600);
}

void loop()
{
  get_gyro();
  get_accel();

  //subtracting the offset of the reading
  Gxyz[0] = Gxyz[0] - G_cal_x;//Roll_gyro
  Gxyz[1] = Gxyz[1] - G_cal_y;//Pitch_gyro

  dt = (millis() - last_tm)/1000;

  //only need the rate of change for the complimentery filter
  Roll_gyro = Gxyz[0]*dt;
  Pitch_gyro = Gxyz[1]*dt;

  last_tm = millis();

  //accel data conversion to euler angles or rate of angles either one
  Roll_accel = (float)atan2(Axyz[1],Axyz[2])*(180/3.1415);
  Pitch_accel = (float)(atan2(-Axyz[0], sqrt((Axyz[1] * Axyz[1]) + (Axyz[2] * Axyz[2]))) * (180 / 3.1415));

  //complimentery filter
  Roll_output = 0.98*(Roll_output+Roll_gyro) + 0.02*Roll_accel;
  Pitch_output = 0.98*(Pitch_output+Pitch_gyro) + 0.02*Pitch_accel;

  PID_compute();

  PWM_motor_1 = value - ((pid_pitch_output + pid_roll_output )*11.1);
  PWM_motor_2 = value + ((pid_pitch_output - pid_roll_output )*11.1);
  PWM_motor_3 = value + ((pid_pitch_output + pid_roll_output )*11.1);
  PWM_motor_4 = value + ((pid_roll_output - pid_pitch_output)*11.1);

  if(PWM_motor_1 < 0) PWM_motor_1 = 0;
  if(PWM_motor_2 < 0) PWM_motor_2 = 0;
  if(PWM_motor_3 < 0) PWM_motor_3 = 0;
  if(PWM_motor_4 < 0) PWM_motor_4 = 0;

  if(PWM_motor_1 > 1900) PWM_motor_1 = 1900;
  if(PWM_motor_2 > 1900) PWM_motor_2 = 1900;
  if(PWM_motor_3 > 1900) PWM_motor_3 = 1900;
  if(PWM_motor_4 > 1900) PWM_motor_4 = 1900;

  if(Roll_output > 90)
  {
    PWM_motor_1 = 0;
    PWM_motor_2 = 0;
    PWM_motor_3 = 0;
    PWM_motor_4 = 0;
  }
  if(Pitch_output > 90)
  {
    PWM_motor_1 = 0;
    PWM_motor_2 = 0;
    PWM_motor_3 = 0;
    PWM_motor_4 = 0;
  }

  if(Serial.available())
  {
    value = Serial.parseInt();
  }

  while(millis() - loop_timer < 20)
  {
    loop_timer = millis();
    Serial.print("I am here");
  }

  // Serial.print(Roll_output);
  // Serial.print(" ");
  // Serial.print(Pitch_output);
  // Serial.print(" ");
  // Serial.print(pid_roll_output);
  // Serial.print(" ");
  // Serial.print(pid_pitch_output);
  // Serial.print(" ");
  // Serial.print(PWM_motor_1);
  // Serial.print(" ");
  // Serial.print(PWM_motor_2);
  // Serial.print(" ");
  // Serial.print(PWM_motor_3);
  // Serial.print(" ");
  // Serial.print(PWM_motor_4);
  // Serial.println();

}

void get_gyro()
{
  accelgyro.getMotion9(&ax,&ay,&az,&gx,&gy,&gz,&mx,&my,&mz);
  Gxyz[0] = gx/131;
  Gxyz[1] = gy/131;
  Gxyz[2] = gz/131;
}

void get_accel()
{
  accelgyro.getMotion9(&ax,&ay,&az,&gx,&gy,&gz,&mx,&my,&mz);
  Axyz[0] = (float)ax/16384;
  Axyz[1] = (float)ay/16384;
  Axyz[2] = (float)az/16384;
}

void PID_compute()
{
  pid_error_temp = Roll_output - setpoint_roll;
  pid_i_term_roll += Ki_roll * pid_error_temp;
  if(pid_i_term_roll > pid_max_roll) pid_i_term_roll = pid_max_roll;
  else if(pid_i_term_roll < (pid_max_roll*(-1))) pid_i_term_roll = ((-1)*pid_max_roll);

  pid_roll_output = (Kp_roll*pid_error_temp) + pid_i_term_roll + (Kd_roll*(pid_error_temp - last_er_tmp_roll));
  if(pid_roll_output > pid_max_roll) pid_roll_output = pid_max_roll;
  else if(pid_roll_output < (pid_max_roll*(-1))) pid_roll_output = pid_max_roll * (-1);

  last_er_tmp_roll = pid_error_temp;

  pid_error_temp = Pitch_output - setpoint_pitch;
  pid_i_term_pitch += Ki_roll * pid_error_temp;
  if(pid_i_term_pitch > pid_max_roll) pid_i_term_pitch = pid_max_roll;
  else if(pid_i_term_pitch < (pid_max_roll*(-1))) pid_i_term_pitch = pid_max_roll * (-1);

  pid_pitch_output = (Kp_roll * pid_error_temp) + pid_i_term_pitch + (Kd_roll*(pid_error_temp-last_er_tmp_pitch));
  if(pid_pitch_output > pid_max_roll) pid_pitch_output = pid_max_roll;
  else if(pid_pitch_output < (pid_max_roll*(-1))) pid_pitch_output = pid_max_roll * (-1);

  last_er_tmp_pitch = pid_error_temp;
}
