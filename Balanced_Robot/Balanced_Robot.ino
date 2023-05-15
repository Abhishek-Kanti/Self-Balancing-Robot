#include <math.h>
#include <Wire.h>

int c = 0;
const int MPU = 0x68;

#define PWM_FREQUENCY 15000

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
float Gyro_Angle, Gyro_Angle_roll, Gyro_Angle_yaw, Acc_Angle, Acc_Angle_roll, Acc_Angle_yaw, Auto_Setpoint;
int Activated; 

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

float theta, phi;

unsigned long millisOld;
float dt;

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float Temp_Error, PID_I, Setpoint, PID_Value, Last_D_Error, PID_D;

#define MOTOR1_PIN1 10
#define MOTOR1_PIN2 9
#define MOTOR1_en 11

#define MOTOR2_PIN1 6
#define MOTOR2_PIN2 7
#define MOTOR2_en 5

float Kp = 5;
float Kd = 1;
float Ki = 0.01;

float setpoint=0.00;
float rollActual;
float error=0;
float errorOld;
float errorChange;
float errorSlope=0;
float errorArea=0;

float PID_output;
float PID_max = 30;
float PID_min = -30;
float MotorSpeed, Motor1Speed, Motor2Speed;
float Comp_Motor_Slack = 60;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  setupMPU();
  calculate_IMU_error();
  
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR1_en, OUTPUT);

  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  pinMode(MOTOR2_en, OUTPUT);

  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
  ICR1 = F_CPU / PWM_FREQUENCY / 2;
  OCR1A = 0;
  OCR1B = 0;

  Temp_Error = 0;
  Setpoint = 0;
  Auto_Setpoint = 0;
  analogWrite(MOTOR1_en,0);
  analogWrite(MOTOR2_en,0);
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
  millisOld = millis();
}


void loop() {
  recordAccelRegisters();
  recordGyroRegisters();

  Acc_Angle = (atan(-1 * accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI) - AccErrorY;
  Acc_Angle_roll = (atan(accelY / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI) - AccErrorX;
  
  if (Activated == 0 && Acc_Angle > -0.5 && Acc_Angle < 0.5) {    //If the accelerometer angle is almost 0
    Gyro_Angle = Acc_Angle;                                       //Load the accelerometer angle in the Gyro_Angle variable
    Gyro_Angle_roll = Acc_Angle;
    Activated = 1;                                                //Set "Activated" variable and start PID control
  }

   dt = (millis() - millisOld) / 1000.;
   millisOld = millis();

  Gyro_Angle += rotY * dt;
  Gyro_Angle_yaw += rotZ * dt;

  //complimentary filter
  Gyro_Angle = 0.98 * Gyro_Angle + 0.02 * (Acc_Angle); //phi
  Gyro_Angle_roll = 0.98 * Gyro_Angle_roll + 0.02 * (Acc_Angle_roll);
  //Serial.println(Gyro_Angle);

  // pid control stats here...
 
  //First, we calculate the error between the real angle and the value taht we want, in this case would be 0º
  Temp_Error = Gyro_Angle - Setpoint - Auto_Setpoint;

  //I value
  if(-10 < Temp_Error < 10){
    PID_I += Ki * (Temp_Error);                                             //Calculate the "I" value
  }
  
  if (PID_I > 400)PID_I = 400;                                              //We limit the "I" to the maximum output
  else if (PID_I < -400)PID_I = -400;


  //Calculate the PID_D output value
  PID_D = Kd * (Temp_Error - Last_D_Error)/dt;

  //Calculate the PID output value
  PID_Value = Kp * Temp_Error + PID_I + PID_D;
  
  if (PID_Value > 400)PID_Value = 400;                                      //Limit the P+I to the maximum output
  else if (PID_Value < -400)PID_Value = -400;

  Last_D_Error = Temp_Error;                                                //Store the error for the next loop

  if (PID_Value < 6 && PID_Value > - 6)PID_Value = 0;                       //Dead-band where the robot is more or less balanced

  if (Gyro_Angle > 30 || Gyro_Angle < -30 || Activated == 0) {              //If the robot falls or the "Activated" is 0
    PID_Value = 0;                                                          //Set the PID output to 0 so the motors are stopped
    PID_I = 0;                                                              //Reset the I-controller memory
    Activated = 0.00;                                                       //Set the Activated variable to 0
    Auto_Setpoint = 0.00;                                                   //Reset the Auto_Setpoint variable
  }

  //  if (PID_Value > PID_max || PID_Value < PID_min) {
  //    Temp_Error += PID_Value * 0.015 ;
  //  }

  //conversion to motor speed;
  MotorSpeed =  (abs(PID_Value)/PID_max)*255;
  if(MotorSpeed > 255)MotorSpeed=255; 

  int pwm_value1 = map(MotorSpeed , 0, 255, 0, ICR1);        // Comment this is you want to use analogWrite function
  //int pwm_value2 = map(abs(MotorSpeed), 0, 255, 0, ICR1);
  OCR1A = pwm_value1;

  //Comp_Motor_Slack = 0.00;
//  if(-6 < Gyro_Angle_yaw <6)Comp_Motor_Slack = 0.00;
//  else if(Gyro_Angle_yaw <= -6)Comp_Motor_Slack = 10.00;
//  else if(Gyro_Angle_yaw >= 6)Comp_Motor_Slack = -10.00;
 
  Motor1Speed = MotorSpeed ;
  Motor2Speed = MotorSpeed - Comp_Motor_Slack;

//  analogWrite(MOTOR1_en, Motor1Speed);                   // Motor speed control using analogWrite function
//  analogWrite(MOTOR2_en, Motor2Speed);

//  Serial.print("Motor1Speed = ");
  Serial.print(Motor1Speed);
//  Serial.print(" | ");
//  Serial.print("Motor2Speed = ");
  Serial.print(Motor2Speed);
//  Serial.print(" | ");
//  Serial.print("Gyro_Angle = ");
//  Serial.println(Gyro_Angle);
//  Serial.print(" | ");
//  Serial.print("PID_Value = ");
//  Serial.println(PID_D);
//  Serial.print(" | ");
//  Serial.print("Temp_Error = ");
//  Serial.println(Temp_Error);
    
    
  if(PID_Value >= 0.8){
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW);
    //Serial.println("forward");
  }
  else if(PID_Value <= -0.8){
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, HIGH);
    //Serial.println("backward");
  }
  else{
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, LOW);
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, LOW);
    //Serial.println("stop");
  }

  Serial.print(255);
  Serial.println(MotorSpeed);
}

void setupMPU() {
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0b00001000); //Setting the accel to +/- 4g
  Wire.endTransmission();
  //some filtering for the raw data
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request Accel Registers (3B - 40)
  while (Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData() {
  gForceX = (accelX / 8192.0);
  gForceY = (accelY / 8192.0);
  gForceZ = (accelZ / 8192.0);
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request Gyro Registers (43 - 48)
  while (Wire.available() < 6);
  gyroX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  rotX = (gyroX / 131.0)-GyroErrorX; //subtracting GyroErrorX
  rotY = (gyroY / 131.0)-GyroErrorY;
  rotZ = (gyroZ/ 131.0)-GyroErrorZ;
}

void calculate_IMU_error() {
  
  AccErrorX = AccErrorY = AccErrorZ = 0;
  GyroErrorX = GyroErrorY = GyroErrorZ = 0;
  c = 0;
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 8192.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 8192.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 8192.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorZ = AccErrorZ + (atan(-1*accelY / accelX) * 180 / PI);
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  AccErrorZ = AccErrorZ / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
 
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
}
