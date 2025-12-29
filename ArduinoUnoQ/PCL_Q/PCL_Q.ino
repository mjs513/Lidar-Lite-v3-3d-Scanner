#include <Arduino.h>
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
#include "LibPrintf.h"
/* ============================================= */

#include <LIDARLite.h>
// Globals
LIDARLite myLIDAR;
/* ============================================= */

#include <Adafruit_PWMServoDriver.h>
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define SERVO_PIN_YAW 0
#define SERVO_PIN_PITCH 1

//Thank you copilot
#define MAP_ROUND(x, in_min, in_max, out_min, out_max) \
  ( ((int64_t)(x) - (int64_t)(in_min)) * ((int64_t)(out_max) - (int64_t)(out_min)) + \
    (((int64_t)(x) - (int64_t)(in_min)) >= 0 ? ((int64_t)(in_max) - (int64_t)(in_min)) / 2 : -((int64_t)(in_max) - (int64_t)(in_min)) / 2) ) \
    / ((int64_t)(in_max) - (int64_t)(in_min)) + (int64_t)(out_min)

uint16_t USMIN[] = {950, 625};
uint16_t USMAX[] = {1870, 2275};
/* ============================================= */

// Minimum and maximum servo angle in degrees
// Modify to avoid hitting limits imposed by pan/tilt bracket geometry
int minPosX = 10;
int maxPosX = 170;
int minPosY = -20;
int maxPosY = +75;
int scanIncrement = 1;

int lastPosX = 0;
int lastPosY = 0;
int loopCount = 0;
int radius = 0;
int lastRadius = 0;
boolean scanning = false;
boolean scanDirection = false;
boolean oldScanDirection = false;

int indexLidar = 0;

int posX = (maxPosX + minPosX) / 2;
int posY = (maxPosY + minPosY) / 4;
float pi = 3.14159265;
float deg2rad = pi / 180.0;
  int distance;
int cal_cnt = 0;
int holdPos = 0;
int pass = 0;

#include "eigen.h"      // Calls main Eigen matrix class library
#include <Eigen/LU>     // Calls inverse, determinant, LU decomp., etc.
using namespace Eigen;  // Eigen related statement; simplifies syntax for declaration of matrices

using namespace Eigen;    // Eigen related statement; simplifies syntax for declaration of matrices
MatrixXi LIDAR(21, 91);


void setup() {
  Serial.begin(115200);
  Wire.begin(); //Join I2C bus
  Wire.setClock(400000);

  delay(2000);
  
  myLIDAR.begin(0, true); // Set configuration to default and I2C to 400 kHz
  myLIDAR.configure(0); // Change this number to try out alternate configurations

  printf("LIDAR acknowledged!\n");

  //servoY.attach(SERVO_PIN_PITCH, 700, 2450); // some motors need min/max setting
  //servoX.attach(SERVO_PIN_YAW, 975, 2050);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  posServos();
  delay(1000);

  oldScanDirection = scanDirection;
  pass = -1;

}

void loop() {
  if(holdPos == 0){
    if(oldScanDirection != scanDirection){
      if (scanDirection) {
        indexLidar = 90;
      } else {
        indexLidar = 0;
      }
      pass += 1;
      oldScanDirection = scanDirection;
    }
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
        holdPos = 1;
        //print_mtxi(LIDAR);
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
      float azimuth;
      if(posX >= 90) {
        azimuth = 90.0f - posX;
      } else {
        azimuth = -(posX - 90.0f);
      }
      azimuth = azimuth;
      float elevation = (90.0 - posY)
;
      double x = radius * sin(elevation*deg2rad) * cos(azimuth*deg2rad);
      double y = radius * sin(elevation*deg2rad) * sin(azimuth*deg2rad);
      double z = radius * cos(elevation*deg2rad); 
      
      //LIDAR(pass, indexLidar) = radius;
      //Serial.println(String(-x, 5) + " " + String(y, 5) + " " + String(-z, 5)+ " " + String(radius, 5));
      //printf("%f, %f, %f, ", x, y , 26.9875 + z);
      printf("%f %f %f %d\n", x, y , z, radius);
      //printf("pass(%d), indexLidar(%d), AZ(%d), EL(%d), (R)%d\n", pass, indexLidar, posX, posY, radius);
      //printf("%d, %f, %f, %d, %d\n", indexLidar, lidar.angle[indexLidar], lidar.el[indexLidar], lidar.range[indexLidar], lidar.quality[indexLidar]);
      //printf("%f, %f, %d\n", azimuth, elevation , radius);

      if(scanDirection) {
        indexLidar = indexLidar - 1;
      } else {
        indexLidar = indexLidar + 1;
      }
      
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
   posX = ( minPosX);
   posY = ( minPosY);
	 pwm.writeMicroseconds(SERVO_PIN_YAW, angle2us(SERVO_PIN_YAW, posX));
   delay(2000);
	 pwm.writeMicroseconds(SERVO_PIN_PITCH, angle2us(SERVO_PIN_PITCH, (90-posY)));
   delay(2000);

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
		pwm.writeMicroseconds(SERVO_PIN_YAW, angle2us(SERVO_PIN_YAW, posX));
    lastPosX = posX;
    moved = true;
  }
  if (posY != lastPosY) {
    delta += abs(posY - lastPosY);
    pwm.writeMicroseconds(SERVO_PIN_PITCH, angle2us(SERVO_PIN_PITCH, 90-posY));
    lastPosY = posY;
    moved = true;
  }
  delay(10);
  return moved;
}

int angle2us(uint8_t servonum, uint16_t angle) {
  return MAP_ROUND(angle, 0, 180, USMIN[servonum], USMAX[servonum]);
}

// PRINT MATRIX (float type)
// By: randomvibe
//-----------------------------
void print_mtxi(const Eigen::MatrixXi& X)  
{
   int i, j, nrow, ncol;
   
   nrow = X.rows();
   ncol = X.cols();

   Serial.print("nrow: "); Serial.println(nrow);
   Serial.print("ncol: "); Serial.println(ncol);       
   Serial.println();
   
   for (i=0; i<nrow; i++)
   {
       for (j=0; j<ncol; j++)
       {
           Serial.print(X(i,j));   // print 6 decimal places
           Serial.print(", ");
       }
       Serial.println();
   }
   Serial.println();
}

