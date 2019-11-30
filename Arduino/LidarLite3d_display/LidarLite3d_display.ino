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
#include "SPI.h"
#include <ArduinoGL.h> 

#define ra8875 1
// For the Adafruit shield, these are the default.
#define TFT_DC  9
#define TFT_CS 10
#define TFT_RST 8

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Arduino_OpenGL tft = Arduino_OpenGL(TFT_CS, TFT_RST);

uint8_t use_fb = 1;

// Color definitions
#define BLACK       0x0000      /*   0,   0,   0 */
#define YELLOW      0xFFE0      /* 255, 255,   0 */
#define WHITE       0xFFFF      /* 255, 255, 255 */

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
int minPosY = 0;
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

int pointCloudCnt = 500;
DMAMEM struct Points {
   float  x;
   float  y;
   float  z;
   int  dist;
} points[500];
uint32_t scanPointCount = 0;

float angleRotY = 6.5f;
float angleRotX = 6.5f;
float angleIncrement = 5;
float scale = 0.05f;
float scaleIncrement = 0.0025f;
float translateX = 0.f;
float translateY = 0.f;
float transIncrement = 10.f;

void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin(); //Join I2C bus
  delay(2000);

  //begin display: Choose from: RA8875_480x272, RA8875_800x480, RA8875_800x480ALT, Adafruit_480x272, Adafruit_800x480
  tft.begin(Adafruit_800x480);
  tft.clearScreen(BLACK);
  tft.setFontScale(1);
  tft.setRotation(0);
  /* Pass the canvas to the OpenGL environment */
  tft.glClear(BLACK);
  tft.glPointSize(0);
    
  tft.glMatrixMode(GL_PROJECTION);
  tft.glLoadIdentity();

  tft.gluPerspective(60.0, (float)tft.width()/(float)tft.height(), 0.1f, 9999.f);
    
  tft.glMatrixMode(GL_MODELVIEW);
  
  //Lidar setup
  myLIDAR.begin(0, true); // Set configuration to default and I2C to 400 kHz
  myLIDAR.configure(0); // Change this number to try out alternate configurations

  Serial.println("LIDAR acknowledged!");

  servoY.attach(SERVO_PIN_PITCH, 700, 2450); // some motors need min/max setting
  servoX.attach(SERVO_PIN_YAW, 1025, 1990);
  posServos();
    
}

void loop() {
  tft.glLoadIdentity();
  tft.gluLookAt(0, 0, -15, 0, 3, 0, 0, 1, 0);
  
  tft.glScalef(0.05f, 0.05f, 0.05f);
  tft.glRotatef(angleRotY, 0.f, 1.f, 0.f);
  
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
      //Serial.println(String(-x, 5) + " " + String(y, 5) + " " + String(-z, 5)+ " " + String(radius, 5));
      if(scanPointCount < pointCloudCnt){
        points[scanPointCount].x = (float) x;
        points[scanPointCount].y = (float) y;
        points[scanPointCount].z = (float) z;
        points[scanPointCount].dist = (int) radius;
        scanPointCount = scanPointCount+1;
      } else if(scanPointCount == pointCloudCnt) {
		  holdPos = 1;
		  updateDisplay();
	  }
      tft.glBegin(GL_POINTS);
      tft.glPointSize(0);
      setPointColor((int) radius);
      tft.glVertex3f(x, -z, -y);
      tft.glEnd();
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
	  scanPointCount = 0;
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

void updateDisplay(){
  while(1){
	  // See if any text entered
	  int ich;
	  if ((ich = Serial.read()) != -1) {
  		if (ich == 'x') {
  		  break;  
  		}
  		if (ich == '6') { // rotate right
  			angleRotY += angleIncrement;  
        drawModel();
  		}
  		if (ich == '4') { // rotate left
  			angleRotY -= angleIncrement; 
        drawModel();
  		}
      if (ich == '8') { // rotate up
        angleRotX += angleIncrement;  
        drawModel();
      }
      if (ich == '2') { // rotate dwn
        angleRotX -= angleIncrement; 
        drawModel();
      }
      if (ich == '9') { // zoom in
        scale += scaleIncrement;  
        drawModel();
      }
      if (ich == '3') { // zoom out
        scale -= scaleIncrement; 
        drawModel();
      }
      if (ich == 'u') { // move up
        translateY += transIncrement;  
        drawModel();
      }
      if (ich == 'm') { // move down
        translateY -= transIncrement; 
        drawModel();
      }
      if (ich == 'h') { // rotate up
        translateX += transIncrement;  
        drawModel();
      }
      if (ich == 'k') { // rotate dwn
        translateX -= transIncrement; 
        drawModel();
      }
      
	  }
	}
}

void drawModel(){
    tft.glClear(BLACK);

	  tft.glLoadIdentity();
	  tft.gluLookAt(0, 0, -15, 0, 3, 0, 0, 1, 0);
	  
	  tft.glScalef(scale, scale, scale);
	  tft.glRotatef(angleRotY, 0.f, 1.f, 0.f);
	  tft.glRotatef(angleRotX,1.f,0.f, 0.f);
    tft.glTranslatef(translateX, translateY, 0.f);

    for(int i = 0; i < pointCloudCnt; i++){
		  tft.glBegin(GL_POINTS);
      tft.glPointSize(0);
      setPointColor(points[i].dist);
		  tft.glVertex3f(points[i].x, -points[i].z, -points[i].y);
		  tft.glEnd();
	  }
}

void setPointColor(int radius){
    if(radius <= 5){
      tft.glColorP(255, 0, 0);
    } else if(radius > 5 && radius <= 10) {
      tft.glColorP(255, 255, 255);
    } else if(radius > 10 && radius <= 20) {
      tft.glColorP(192, 192,192);
    } else if(radius > 20 && radius <= 30) {
      tft.glColorP(204, 204,255);
    } else if(radius > 30 && radius <= 60) {
      tft.glColorP(255, 255,153);//
    } else if(radius > 60 && radius <= 90) {
      tft.glColorP(255, 204,153);
    } else if(radius > 90 && radius <= 150) {
      tft.glColorP(204, 255,255);
    } else if(radius > 150 && radius <= 310){
      tft.glColorP(255, 255, 204);
    } else if(radius > 310 && radius <= 500){
      tft.glColorP(153, 153, 255);
    } else {
      tft.glColorP(153, 51, 102);
      //tft.glColorP(0, 0, 0);
    }
}
