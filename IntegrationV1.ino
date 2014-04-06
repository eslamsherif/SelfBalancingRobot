
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <PID_v1.h>

//Integration Variables
int STD_LOOP_TIME = 9;
unsigned long loopStartTime = 0;
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
double actAngle;

// MPU6050 Variables

// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz; 

//Kalman Filter Variables
float Q_angle  =  0.001;
float Q_gyro   =  0.003;
float R_angle  =  0.03;

float x_angle = 0;
float x_bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;	
float dt, y, S;
float K_0, K_1;

//l298
double Speed = 0;
char Dir = 'f';
const int IN1 = 7;
const int IN2 = 8;
const int EN = 9;

//PID Variables
//********************Change the tuning parameters here**********************
//Setpoint.  Find where the robot is balanced.  
double Setpoint = -170;
//Point where it switches from conservative to agressive 
int gapDist = 15;
//Aggressive
double aggKp = 2, aggKi = 0.12, aggKd = 1.2;
//Conservative
double consKp = 1.2, consKi = 0.08, consKd = 0.75;
//***************************************************************************
PID myPID(&actAngle, &Speed, &Setpoint, consKp, consKi, consKd, DIRECT);

//#define LED_PIN 13
//bool blinkState = false;

void setup() {
    Wire.begin();
    
    //Serial.begin(38400);

    // initialize device
    //Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
    //Configure PID
    myPID.SetSampleTime(1);
    myPID.SetMode(AUTOMATIC);
    
//    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//    actAngle = kalmanCalculate(ax, gx, lastLoopTime);
//    
//    Setpoint = actAngle;
    
    // configure Arduino LED for
    //pinMode(LED_PIN, OUTPUT);
    pinMode(EN, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    actAngle = kalmanCalculate(ax, gx, lastLoopTime);
    
    //Serial.print("a/g:\t");
    //Serial.print(actAngle);  Serial.print("\t");
    
    double gap = Setpoint-actAngle; //distance away from setpoint
    double absgap = abs(gap);
    
    //Serial.print(gap);  Serial.print("\t");
    
    if(absgap < gapDist)
    {  
      //we're close to setpoint, use conservative tuning parameters
      myPID.SetTunings( consKp, consKi, consKd);
    }
    else
    {
       //we're far from setpoint, use aggressive tuning parameters
      myPID.SetTunings( aggKp, aggKi, aggKd);
    }
    
    myPID.Compute();
    
    if( gap >= 0 ) {
      Dir = 'r';
    }
    else if ( gap < 0 ) {
      Dir = 'f';
    }
    
    map(Speed, 0, 65535 , 0, 255);
    
    //Serial.println(Speed);
         
    Drive_Motor( Speed, Dir); 
    
      // *********************** loop timing control **************************
    lastLoopUsefulTime = millis() - loopStartTime;
    
//    if( lastLoopUsefulTime < STD_LOOP_TIME )
//      delay( ( STD_LOOP_TIME - lastLoopUsefulTime ) );
    
    lastLoopTime = millis() - loopStartTime;
    loopStartTime = millis();

    // blink LED to indicate activity
    //blinkState = !blinkState;
    //digitalWrite(LED_PIN, blinkState);
}

float kalmanCalculate(float newAngle, float newRate,int looptime) {
    
    dt = float(looptime)/1000;
    x_angle += dt * (newRate - x_bias);
    P_00 +=  dt * ( dt * P_11 - P_01 - P_10 + Q_angle);
    P_01 -=  dt * P_11;
    P_10 -=  dt * P_11;
    P_11 +=  + Q_gyro * dt;
    
    y = newAngle - x_angle;
    S = P_00 + R_angle;
    K_0 = P_00 / S;
    K_1 = P_10 / S;
    
    x_angle +=  K_0 * y;
    x_bias  +=  K_1 * y;
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;
    
    return x_angle;
}  //Kalman

void Drive_Motor( double Speed, char Dir) {
  
  if(Dir == 'f') { //Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if( Dir == 'r' ) {  //reverse motion
    digitalWrite (IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  
  analogWrite(EN, (int) Speed);
  
}  //Drive_Motor
