// 0.0 General Includes and declarations
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#define G 16384.0      //Both Values obtained from MPU6050 Datasheet
#define GYRO_SCL 131.0

// 1.X Interface PINS and Modules Variables
// 1.0 Timing Variables
uint32_t loopStartTime;

// 1.1 Motor PINS and variables   //Both Motors are drived together
const int IN1 = 7;  //Input 1
const int IN2 = 8;  //Input 2
const int EN = 9;   //Enable
double Speed = 0;
char Dir = 'f';

//1.2 MPU6050 PINS  
/*
  Pin 1 = 3.3V
  Pin 2 = GND
  Pin 3 = A5
  Pin 4 = A4
*/

//1.3 Output Led Pin and Variables
#define LED_PIN 13
bool blinkState = false;

//1.4 MPU6050 Variables
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
double acc_angle;
double gyro_rate;

//1.5 Kalman Filter Variables
float Q_angle  =  0.001;
float Q_gyro   =  0.003;
float R_angle  =  0.03;

double actAngle; //Output Angle
double angle = 0;  // Reset the angle
float bias = 0;   // Reset bias
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;  //error covirance matrix reset to 0
float dt, y, S;
float K_0, K_1;

//1.6 PID Variables
//Setpoint. where the robot is balanced.  
double Setpoint = 0;
//distance away from setpoint
double error, ITerm, lastInput, dInput;
double Kp = 30.0, Ki = 0.0, Kd = 4.5;
unsigned long SampleTime = 100;
unsigned long lastTime;
double outMin = -255.0, outMax = 255.0;
double output; //Temp Var for debugging

//3.0 Implementation Code
//3.1 Setup Intialization Code 
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    
    // initialize serial communication
    Serial.begin(38400);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    accelgyro.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(accelgyro.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    loopStartTime = millis();
    
    SetSampleTime(5);
    lastTime = millis()-SampleTime;
    
    //configure Arduino Pins
    pinMode(EN, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
}  //setup

//3.2 Execution Loop
void loop() {
  // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);    //Debug_Raw();
    
    acc_angle = getAccAng(ay, az);
    gyro_rate = gx / GYRO_SCL;    //Debug_Orient();
    
    actAngle = kalmanCalculate( acc_angle, gyro_rate, millis() - loopStartTime );
    loopStartTime = millis();    //Debug_Filtered();
    
    error = Setpoint - actAngle; //distance away from setpoint
    
    if( error > 0 ) {
      Dir = 'f';
    }
    else if ( error < 0 ) {
      Dir = 'r';
    }
    
    Compute();    
    
    if(Speed < 75)
      Speed = 0;    //To Stop Whinning Sound of the Motors as Low Speeds
    
    if ( actAngle < -30 || actAngle > 30)
      Speed = 0;    //No point of Trying to balance the robot if it is after 30 degrees
    
//    Debug_PID();
    
    Drive_Motor( Speed, Dir); 
    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}  //loop

//4.X Additional Function
//4.1 Calculate Acc Angle
inline double getAccAng(int16_t ay, int16_t az) {
    // Convert to 360 degrees resolution
    // atan2 outputs the value of -π to π (radians)
    // We are then convert it to 0 to 2π and then from radians to degrees
    double angle = ( ( atan2( -ay , -az ) + PI ) * RAD_TO_DEG );
    
    if( angle >= 180 ) {  //to map the range from -180 to 180
      angle -= 360;
    }
    return angle;
}

//4.2 Kalman Function
double kalmanCalculate(float newAngle, float newRate,int looptime) {
    
    dt = looptime / 1000.0;
    angle += dt * (newRate - bias);  //angle = rate * timesample
    
    P_00 +=  dt * ( dt * P_11 - P_01 - P_10 + Q_angle);
    P_01 -=  dt * P_11;
    P_10 -=  dt * P_11;
    P_11 +=  Q_gyro * dt;
    
    S = P_00 + R_angle;
    K_0 = P_00 / S;
    K_1 = P_10 / S;
    
    y = newAngle - angle;    
    angle +=  K_0 * y;
    bias  +=  K_1 * y;
    
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;
    
    return angle;
}  //Kalman

//4.3 Drive Motors
void Drive_Motor( double Speed, char Dir) {
  
  if(Dir == 'f') { //Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if( Dir == 'r' ) {  //reverse motion
    digitalWrite (IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  
  analogWrite(EN, (int) Speed);  //PWM is written last to ensure the Motor move in the
                                 //correct direction
  
}  //Drive_Motor

void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      Ki *= ratio;
      Kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

void Compute()
{
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange >= SampleTime)
   {
      /*Compute all the working error variables*/     
      ITerm += (Ki * error);
      
      if(ITerm > outMax) ITerm= outMax;      //To reduce the time ITerm needs to settle down after error was recovered 
      else if(ITerm < outMin) ITerm= outMin;
      
      dInput = (actAngle - lastInput);
      
      /*Compute PID Output*/
      output = Kp * error + ITerm - Kd * dInput;
      
      if(output > outMax) output = outMax;
      if(output < outMin) output = outMin;
      if(output < 0) output = abs(output);
      
      Speed = output;
	  
      /*Remember some variables for next time*/
      lastInput = actAngle;
      lastTime = now;
   }
}

////5.X Debug Functions
////5.1 Printing MPU6050 Output Raw Values
////TODO: More Style Formating
//void Debug_Raw() {
//    // display tab-separated accel/gyro x/y/z values
//    Serial.print(F("MPU Raw Readings: "));
//    Serial.print(ax); Serial.print("\t");
//    Serial.print(ay); Serial.print("\t");
//    Serial.print(az); Serial.print("\t");
//    Serial.print(gx); Serial.print("\t");
//    Serial.print(gy); Serial.print("\t");
//    Serial.println(gz);
//}
//
//void Debug_Orient() {
//    Serial.print(F("Robot Orientation: "));
//    Serial.print(acc_angle); Serial.print("\t");
//    Serial.println(gyro_rate);
//}
//
//void Debug_Filtered() {
//    Serial.print(F("Robot Actual Angle: "));
//    Serial.print(F("Angle:"));         Serial.println(actAngle);
//}
//
//void Debug_PID() {
//    Serial.print(F("PID: "));
//    Serial.print(F("Angle:"));         Serial.print(actAngle);       Serial.print("\t");
//    Serial.print(F("Error:"));         Serial.print(error);          Serial.print("\t");
//    Serial.print(F("Proportional:"));  Serial.print(Kp * error);     Serial.print("\t");
//    Serial.print(F("Integral:"));      Serial.print(ITerm);          Serial.print("\t");
//    Serial.print(F("Derivative:"));    Serial.print(Kd * dInput);    Serial.print("\t");
//    Serial.print(F("Output:"));        Serial.print(output);         Serial.print("\t");
//    Serial.print(F("Dir:"));           Serial.print(Dir);            Serial.print("\t");
//    Serial.print(F("Speed:"));         Serial.println(Speed);
//}
//
//void Debug_Motion() {
//    Serial.print(F("Robot Motion: "));
//    Serial.print(F("Angle:"));         Serial.print(actAngle);       Serial.print("\t");
//    Serial.print(F("Dir:"));           Serial.print(Dir);            Serial.print("\t");
//    Serial.print(F("Speed:"));         Serial.println(Speed);
//}
