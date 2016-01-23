//#include <helper_3dmath.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" // gyro
#include <SoftwareSerial.h>
#include <Wire.h>
#include <stdio.h>    
#include <stdlib.h>
#include <Servo.h>

#define OUTPUT_READABLE_YAWPITCHROLL; //OUTPUT_READABLE_WORLDACCEL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)


int inPin   = 0;
int inPin2  = 1;

Servo myservoH;  // create servo object to control a servo
Servo myservoV;  // create servo object to control a servo




int offset = 16;
int servoMin = 0;
int servoMax = 180;
int servoH = servoMax/2;
int servoV = servoMax/2; 

// GYROSKOP

MPU6050 mpu;
bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffe
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

int16_t ax, ay, az;
int16_t gx, gy, gz;
int val;
int prevVal;


void setup() {
  Wire.begin();
  Serial.begin(9600);  
 
  gyroSetup();
  //myservoV.attach(9);  // attaches the servo on pin 9 to the s ervo object
  myservoH.attach(8, 400, 2600);
  //myservoH.setMaximumPulse(2000);
  //myservoH.setMinimumPulse(700);
  
  //pinMode(inPin, OUTPUT);
  //pinMode(inPin2, OUTPUT);
}

int i = 0;
int pos=1500;

void loop() {
 readMethod();
}



void readMethod(){
  // funkcja sczytuje dane wyslane od komputera i odpala poszczegolne funkcje
 char lastline[50];  int k=0;  
  if (Serial.available() > 0) 
  { 
   while (Serial.available() > 0) {
      lastline[k]=Serial.read(); //read data  
      k++;      
   }
   lastline[k]='\0';
   Serial.println(lastline);
    if(strcmp(lastline, "forward()") == 0){
      forward();
    }
    else if(strcmp(lastline, "backward()") == 0){
      backward();
    }
    else if(strcmp(lastline, "stopEngine()") == 0){
      stopEngine();
    }
    else if(strcmp(lastline, "cameraLeft()") == 0){
      cameraLeft();
    }
    else if(strcmp(lastline, "cameraRight()") == 0){
      cameraRight();
    }
    else if(strcmp(lastline, "cameraUp()") == 0){
      cameraUp();
    }
    else if(strcmp(lastline, "cameraDown()") == 0){
      cameraDown();
    }
    else if(strcmp(lastline, "getSensorsData()") == 0){
      getSensorsData();
    }
    else if(strcmp(lastline, "setLight1()") == 0){
      setLight(1);
    }
    else if(strcmp(lastline, "setLight2()") == 0){
      setLight(2);
    }
    else if(strcmp(lastline, "setLight3()") == 0){
      setLight(3);
    } 
    else if(strcmp(lastline, "getGyro()") == 0){
      gyro(); 
    }
    
  } 
}

void forward(){
   Serial.println(F("procedura *forward()* zostala rozpoczeta")); 
}

void backward(){
   Serial.println(F("procedura *bakward()* zostala rozpoczeta"));  
}

void stopEngine(){

}

void cameraRight(){
   Serial.println(F("procedura *cameraRight()* zostala rozpoczeta"));  
   
   if(servoH<servoMax){
     servoH += offset;
   }else{
     servoH = servoMax ;
   }
   myservoH.write(servoH);
}

void cameraLeft(){
   Serial.println(F("procedura *cameraLeft()* zostala rozpoczeta"));  
   if(servoH>servoMin ){
     servoH -= offset;
   }else{
     servoH =servoMin  ;
   }
   myservoH.write(servoH);
}

void cameraDown(){
   Serial.println(F("procedura *cameraDown()* zostala rozpoczeta"));  
   if(servoV<servoMax){
     servoV += offset;
   }else{
     servoV = servoMax;
   }
   myservoV.write(servoV); 
}
void cameraUp(){
   Serial.println(F("procedura *cameraUp()* zostala rozpoczeta"));  
   if(servoV>servoMin ){
     servoV -= offset;
   }else{
     servoV = servoMin ;
   }
   myservoV.write(servoV); 
}
void getSensorsData(){
   Serial.println(F("procedura *getSensorsData()* zostala rozpoczeta"));  
}
void setLight(int i){
   Serial.println(F("procedura *setLight()* zostala rozpoczeta")); 
}






























void gyroSetup(){
   
    Wire.begin();
    Serial.begin(38400);
 
    Serial.println("Initialize MPU");
    mpu.initialize();
    Serial.println(mpu.testConnection() ? "Connected" : "Connection failed");
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688); // 1688 factory default for my test chip
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void gyro(){
   if (!dmpReady) return;
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

   }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));
        delay(5);
        gyro();
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif
    } 
}
