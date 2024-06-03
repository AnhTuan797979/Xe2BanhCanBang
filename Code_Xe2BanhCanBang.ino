#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "SimpleKalmanFilter.h" // Thêm thư viện bộ lọc Kalman vào đây
#include <SoftwareSerial.h>



#include <Kalman.h> 
#define RESTRICT_PITCH 
Kalman kalmanX; 
//Kalman kalmanY;
/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
double gyroXangle, gyroYangle; 
double compAngleX, compAngleY; 
double kalAngleX, kalAngleY; 



uint32_t timer;
uint8_t i2cData[14]; 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 50    //50
SimpleKalmanFilter locnhieu(2, 2, 0.01);  //2 2 0.01
float gt_filter;
MPU6050 mpu;
SoftwareSerial Bluetooth(3,4); // TX, RX
// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[2];

// PID
double originalSetpoint = -2.8;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.3;
double input, output;
double Kp = 35;     //35
double Kd = 1.5;      //2.2
double Ki = 250;    //300
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

char data;  
int spd=255;

double motorSpeedFactorLeft = 0.98;        //0.9
double motorSpeedFactorRight = 1;      //0.92

// MOTOR CONTROLLER
int ENA = 10;
int IN1 = 8;  
int IN2 = 9;
int IN3 = 12;
int IN4 = 13;
int ENB = 11;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
    Bluetooth.begin(9600);
    Serial.begin(9600);
    Serial.println("ENTER AT Commands:");
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#endif

  mpu.initialize();

  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);


  if (devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop()
{
  // giao tiếp hc05
  //Đọc dữ liệu từ HC-05 gửi đến Arduino
  if (Bluetooth.available()){
    data=Bluetooth.read();
    //Serial.write(data);
  
    if(data=='F')    // xe tien 
      {
        setpoint= -1.7;
      }
    else if(data=='B')  // xe lui
      {
        setpoint= -3.6;  
      }
    else if(data=='R')//////Rẽ trai
    {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENA,spd-50);   // giảm tốc độ khi quay 
      analogWrite(ENB,spd-50);
    }
    else if(data=='L')//////Rẽ phải
    { 
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENA,spd-50);
      analogWrite(ENB,spd-50);
    }
    else if(data=='V'|| data=='v')
    {
      setpoint=-2.8;   //cho xe can bang 
    }
    else if(data=='X'||data=='x')  // XE DUNG
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(ENA,0);
      analogWrite(ENB,0); 
    }
  }
  //end hc05



  if (!dmpReady)
    return;

  while (!mpuInterrupt && fifoCount < packetSize)
  {
    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // Sử dụng bộ lọc Kalman cho dữ liệu góc nghiêng
    //gt_filter = locnhieu.updateEstimate(ypr[2] * 180 / M_PI);
    //gt_filter= ypr[2] * 180 / M_PI;
    input = -ypr[2] * 180/M_PI;
    // Sử dụng dữ liệu đã lọc để tính toán PID
    //input = locnhieu.updateEstimate(ypr[2] * 180 / M_PI);
    //Serial.println(input); // Serial.print("\t");
    //Serial.println(gt_filter);  //Serial.print("\t");
    Serial.println(data);

  }
}