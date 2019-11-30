// LidarScanner.ino Arduino sketch
// http://www.qcontinuum.org/lidar

// Load sketch into Arduino software, available from:
// https://www.arduino.cc/

// This sketch controls X and Y servos to pan/tilt a LIDAR detector,
// either manually (by pressing LCD buttons to control XY location),
// or automatically (scanning horizontally and vertically).
// XYZ coordinates are output to the serial port to be received and
// displayed on computer by LidarScanner.pde Processing sketch.

// This sketch requires library "LIDAR-Lite v3" by Garmin.
// Select menu "Sketch", "Include Library", "Manage Libraries...",
// and in textbox "Filter your search...", enter "lidar".

// Freetronics LCD shield uses D3 to control backlight brightness,
// but digital output needed for servo control, so disable the
// backlight control by cutting strap marked "D3" on LCD shield.
#include <Wire.h>
#include <LIDARLite.h>

// Globals
LIDARLite myLIDAR;

#include <PWMServo.h>
PWMServo servoX;  // create servo object to control a servo
PWMServo servoY;  // create servo object to control a servo
#define SERVO_PIN_YAW 5
#define SERVO_PIN_PITCH 4

// Minimum and maximum servo angle in degrees
// Modify to avoid hitting limits imposed by pan/tilt bracket geometry
int minPosX = 0;
int maxPosX = 180;
int minPosY = -15;
int maxPosY = 90;


int lastPosX = 0;
int lastPosY = 0;
int loopCount = 0;
int radius = 0;
int lastRadius = 0;
boolean scanning = false;
boolean scanDirection = false;
int scanIncrement = 1;
int posX = (maxPosX + minPosX) / 2;
int posY = (maxPosY + minPosY) / 4;
float pi = 3.14159265;
float deg2rad = pi / 180.0;
  int distance;
int cal_cnt = 0;
int holdPos = 0;

void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin(); //Join I2C bus
  delay(2000);
  
  myLIDAR.begin(0, true); // Set configuration to default and I2C to 400 kHz
  myLIDAR.configure(0); // Change this number to try out alternate configurations

  Serial.println("LIDAR acknowledged!");

  servoY.attach(SERVO_PIN_PITCH, 700, 2450); // some motors need min/max setting
  servoX.attach(SERVO_PIN_YAW, 1025, 1990);
  posServos();
    
}

void loop() {
  if(holdPos == 0){
    if (scanDirection) {
      posX -= scanIncrement;
    } else {
      posX += scanIncrement;
    }
    if (posX > maxPosX || posX < minPosX) {
      // hit limit X limit, reverse auto scan direction
      scanDirection = !scanDirection;
      posY += scanIncrement;
      if (posY > maxPosY) {
        // completed auto scan, return to manual scan mode
        scanning = false;
      }
    }

    posX = min(max(posX, minPosX), maxPosX);
    posY = min(max(posY, minPosY), maxPosY);
    bool moved = moveServos();

    loopCount += 1;
   
    if (loopCount % 100 == 0) {
      // recalibrate scanner every 100 scans
      radius = myLIDAR.distance();
    } else {
      radius = myLIDAR.distance(false);
    }
    if (abs(radius - lastRadius) > 2)
    {
      lastRadius = radius;
      //Serial.print(radius / 100.0,2);
    }
    if (scanning || moved) {
      float azimuth = posX * deg2rad;
      float elevation = (180 - maxPosY + posY) * deg2rad;
      double x = radius * sin(elevation) * cos(azimuth);
      double y = radius * sin(elevation) * sin(azimuth);
      double z = radius * cos(elevation);
      Serial.println(String(-x, 5) + " " + String(y, 5) + " " + String(-z, 5)+ " " + String(radius, 5));
    }
  }
  // See if any text entered
  int ich;
  if ((ich = Serial.read()) != -1) {
    if (ich == 't') {
      holdPos = 1;  
    }
    if (ich == 'r'){
      posServos();
      holdPos = 0;
    }
  }
}


void posServos(){
   posX = (maxPosX + minPosX) / 2;
   posY = (maxPosY + minPosY) / 4;
   servoX.write(posX);
   servoY.write(posY);
   delay(3000);

    // switch to auto scan mode
    scanning = true;
    posX = minPosX;
    posY = minPosY;
    scanDirection = true;
}

bool moveServos()
{
  bool moved = false;
  static int lastPosX;
  static int lastPosY;
  int delta = 0;
  if (posX != lastPosX) {
    delta += abs(posX - lastPosX);
    servoX.write(posX);
    lastPosX = posX;
    moved = true;
  }
  if (posY != lastPosY) {
    delta += abs(posY - lastPosY);
    servoY.write(90-posY);
    lastPosY = posY;
    moved = true;
  }
  delay(50);
  return moved;
}
