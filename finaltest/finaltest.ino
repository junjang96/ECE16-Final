/* 
 *  General Includes
 */
#include <AltSoftSerial.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* 
 *  Global Variables
 */
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

AltSoftSerial BTserial;
const int buttonPin = 4;
const int ir_in = A2;
const int MPU_addr=0x68;  // I2C address of the MPU-6050, can be changed to 0x69
MPU6050 IMU(MPU_addr);
const int interruptPin = 2;
volatile bool ipinReady = false;
int16_t ax, ay, az, tp, gx, gy, gz;
int val_ir, HR, stepCount, buttonState, lastState;

unsigned long samplePeriod = 40000; // 25 Hz for the BLE limitations
unsigned long startTime = 0;
unsigned long buttonDown = 0;
unsigned long lastDown = 0;
unsigned long buttonThresh = 1000;
unsigned long volatile elapsedTime = 0;
unsigned long volatile currentTime = 0;
unsigned long volatile lastTime = 0;
bool newRead = false;
bool sending = false;
bool awake = true;
bool craving = false;
bool ask = false;
bool newAnswer = false;

/* 
 *  Function to check the interrupt pin to see if there is data available in the MPU's buffer
 */
void interruptPinISR() {
  ipinReady = true;
}

/*
 *  Function to read a single sample of IR data
 */
void readIR() {
  val_ir = analogRead(ir_in);
 }
 
/* 
 *  Function to read a single sample of IMU data
 *  NOTE: ONLY READ WHAT YOU INTEND TO USE IN YOUR ALGORITHM!
 */
void readIMU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);                    // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  
  //Accelerometer (3 Axis)
  ax=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  ay=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  //Temperature
  tp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  
  //Gyroscope (3 Axis)
  gx=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

// Function to print step count to OLED
void printOLED() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("Steps: ");
  display.println(stepCount);
  display.print("Heartrate: ");
  display.println(HR);
  
  if (ask && !newAnswer) {
    display.println("Are you craving a smoke?");
    display.println("Click Y | Hold N");
  }
  else if (ask && newAnswer) {
    newAnswer = false;
  }
  display.display();
}

/* 
 *  Function to poll to see if data is available to read
 *  'samplePeriod' defines how frequently we poll... default set to 25Hz
 */
void pollData() {

  if (ipinReady && !newRead) {
  
    currentTime = micros();
    if (currentTime - lastTime >= samplePeriod) {
      elapsedTime = (currentTime - startTime)/1e3;
      lastTime = currentTime;

      readIMU();
      readIR();
      newRead = true;
    }
  }  
}

/*
 *  Function to read button presses and holds
 */
void buttonRead() {
  unsigned long betweenPress, buttonUp;
  lastState = buttonState;
  buttonState = digitalRead(buttonPin);
  
  if ((buttonState == 0) && (lastState == 1)) {
    if (ask) {
      buttonDown = millis();
    }
    else {
      // If button is pressed & not in ASK mode
      awake = !awake;
    }
  }
  if ((buttonState == 1) && (lastState == 0)) {
    if (ask) {
      buttonUp = millis();
      betweenPress = buttonUp - buttonDown;
      if (betweenPress < buttonThresh) {
        // This should trigger NO (BUTTON CLICKED)
        craving = false;
      }
      if (betweenPress >= buttonThresh) {
        // This should trigger YES (BUTTON HELD DOWN)
        craving = true;
      }
      ask = false;
      newAnswer = true;
    }
  }
}

/* 
 *  Function to send data to the Python processing
 *  NOTE: MODIFY ACCORDING TO YOUR ALGORITHM!
 */
void sendData() {
  // Displays the raw value and the time it was sampled
  BTserial.print(elapsedTime);
  BTserial.print(' ');
  BTserial.print(ay);
  BTserial.print(' ');
  BTserial.println(val_ir);
}

/* 
 *  Function to do the usual Arduino setup
 */
void setup(){

  // Intialize the IMU and the DMP ont he IMU
  IMU.initialize();
  IMU.dmpInitialize();
  IMU.setDMPEnabled(true);
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(MPU_addr);   // PWR_MGMT_1 register
  Wire.write(0);          // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // initialize with the I2C addr 0x3C (for the 128x32)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  printOLED();
  
  Serial.begin(9600);
  BTserial.begin(9600); 

  // Create an interrupt for pin2, which is connected to the INT pin of the MPU6050
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), interruptPinISR, RISING);

  // Initialize button
  pinMode(buttonPin, INPUT_PULLUP);
  buttonState = digitalRead(buttonPin);

  // Start time of the Arduino (for elapsed time)
  startTime = micros();
  // Set stepCount to zero
  stepCount = 0;
  HR = 0;
}

/* 
 *  Function to loop continuously: poll ==> send ==> read
 *  NOTE: MODIFY TO SUIT YOUR ALGORITHM!
 */
void loop(){
  // no longer using an ISR for the sake of AltSoftSerial
  pollData();
  buttonRead();
  
//  if (Serial.available() > 0) {
//    int i = Serial.read();
//    Serial.println(i);
//    if (i == '!')
//      ask = true;
//  }

  if (BTserial.available() > 0) { 
    int incomingByte = BTserial.read();
    switch (incomingByte) {
      case '*':
        ask = true;
        BTserial.println("DETECTED");
        break;
      case '#':
        stepCount++;
        break;
      case '(':
        sending = true;
        break;
      case ')':
        sending = false;
        break;
      case '$':
        String dataFromPython =  BTserial.readStringUntil('>'); // I assume data points are separated by commas, but anything is fine
        Serial.print("Received: ");
        Serial.println(dataFromPython);
        HR = dataFromPython.toInt();
        break;
      default:
        sending = false;
        break;
    }
  }
  if (newRead && sending && awake) {
    sendData();
    newRead = false;
  }
  printOLED();
}
